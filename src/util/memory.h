#ifndef MARLON_UTIL_MEMORY_H
#define MARLON_UTIL_MEMORY_H

#include <cstddef>
#include <cstdlib>

#include <bit>
#include <functional>
#include <new>
#include <type_traits>
#include <utility>

#include "size.h"

namespace marlon {
namespace util {
struct Const_block {
  std::byte const *begin;
  std::byte const *end;

  Const_block() = default;

  constexpr Const_block(std::byte const *begin, Size size) noexcept
      : Const_block{begin, begin + size} {}

  constexpr Const_block(std::byte const *begin, std::byte const *end) noexcept
      : begin{begin}, end{end} {}

  constexpr Size size() const noexcept { return end - begin; }
};

struct Block {
  std::byte *begin;
  std::byte *end;

  Block() = default;

  constexpr Block(std::byte *begin, Size size) noexcept
      : Block{begin, begin + size} {}

  constexpr Block(std::byte *begin, std::byte *end) noexcept
      : begin{begin}, end{end} {}

  constexpr operator Const_block() const noexcept { return {begin, end}; }

  constexpr Size size() const noexcept { return end - begin; }
};

// constexpr Const_block make_block(std::byte const *begin,
//                                  std::byte const *end) noexcept {
//   return {begin, end};
// }

// constexpr Block make_block(std::byte *begin, std::byte *end) noexcept {
//   return {begin, end};
// }

// constexpr Const_block make_block(std::byte const *begin, Size size) noexcept
// {
//   return {begin, begin + size};
// }

// constexpr Block make_block(std::byte *begin, Size size) noexcept {
//   return {begin, begin + size};
// }

template <Size Len, Size Align> class Storage {
public:
  std::byte const *data() const noexcept { return _data.data(); }

  std::byte *data() noexcept { return _data.data(); }

  Const_block block() const noexcept { return {data(), data() + Len}; }

  Block block() noexcept { return {data(), data() + Len}; }

private:
  alignas(Align) std::array<std::byte, Len> _data;
};

template <typename T> using Object_storage = Storage<sizeof(T), alignof(T)>;

constexpr Size align(Size size, Size alignment) noexcept {
  return (size + alignment - 1) & -alignment;
}

inline Size ptrdiff(void const *p1, void const *p2) noexcept {
  return std::bit_cast<std::uintptr_t>(p1) - std::bit_cast<std::uintptr_t>(p2);
}

template <Size Alignment = alignof(std::max_align_t)> class Stack_allocator {
public:
  static constexpr Size
  memory_requirement(std::initializer_list<Size> allocation_sizes) noexcept {
    auto result = Size{};
    for (auto const size : allocation_sizes) {
      result += align(size, Alignment);
    }
    return result;
  }

  constexpr Stack_allocator() noexcept : _block{}, _top{} {}

  explicit Stack_allocator(Block block) noexcept
      : _block{block}, _top{block.begin} {}

  constexpr Stack_allocator(Stack_allocator &&other) noexcept
      : _block{std::exchange(other._block, Block{})},
        _top{std::exchange(other._top, nullptr)} {}

  constexpr Stack_allocator &operator=(Stack_allocator &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  Const_block block() const noexcept { return _block; }

  Block alloc(Size size) {
    auto const block_end = _top + size;
    auto const aligned_block_end = _top + align(size, Alignment);
    if (aligned_block_end <= _block.end) {
      auto const result = Block{_top, block_end};
      _top = aligned_block_end;
      return result;
    } else {
      throw std::bad_alloc{};
    }
  }

  void free(Const_block block) noexcept {
    auto const aligned_block_size = align(block.size(), Alignment);
    auto const aligned_block_end = block.begin + aligned_block_size;
    if (aligned_block_end == _top) {
      _top = const_cast<std::byte *>(block.begin);
    }
  }

  bool owns(Const_block block) const noexcept {
    return block.begin >= _block.begin && block.begin < _block.end;
  }

private:
  constexpr void swap(Stack_allocator &other) noexcept {
    std::swap(_block, other._block);
    std::swap(_top, other._top);
  }

  Block _block;
  std::byte *_top;
};

template <class Parent, Size MinSize, Size MaxSize = MinSize>
class Free_list_allocator {
public:
  Free_list_allocator() = default;

  explicit Free_list_allocator(Parent const &parent) : _parent{parent} {}

  explicit Free_list_allocator(Parent &&parent) : _parent{std::move(parent)} {}

  Parent const &parent() const noexcept { return _parent; }

  Block alloc(Size size) {
    if (size >= MinSize && size <= MaxSize) {
      if (_root) {
        auto const result = Block{reinterpret_cast<std::byte *>(_root), size};
        _root = _root->next;
        return result;
      } else {
        return Block{_parent.alloc(MaxSize).begin, size};
      }
    } else {
      return _parent.alloc(size);
    }
  }

  void free(Const_block block) noexcept {
    auto const size = block.size();
    if (size >= MinSize && size <= MaxSize) {
      _root = new (const_cast<std::byte *>(block.begin)) Node{.next = _root};
    } else {
      _parent.free(block);
    }
  }

private:
  struct Node {
    Node *next;
  };

  Parent _parent;
  Node *_root{};

  static_assert(MaxSize >= sizeof(Node));
};

template <Size MinSize, Size MaxSize = MinSize> class Pool_allocator {
public:
  static constexpr Size memory_requirement(Size max_allocations) {
    return MaxSize * max_allocations;
  }

  Pool_allocator() noexcept = default;

  explicit Pool_allocator(Block block) noexcept
      : _impl{Stack_allocator<1>{block}} {}

  Const_block block() const noexcept { return _impl.parent().block(); }

  Size max_blocks() const noexcept { return block().size() / MaxSize; }

  Block alloc(Size size) { return _impl.alloc(size); }

  void free(Const_block block) noexcept { _impl.free(block); }

private:
  Free_list_allocator<Stack_allocator<1>, MinSize, MaxSize> _impl;
};

template <Size MinSize, Size MaxSize = MinSize, typename Allocator>
std::pair<Block, Pool_allocator<MinSize, MaxSize>>
make_pool_allocator(Allocator &allocator, Size max_blocks) {
  auto const block = allocator.alloc(
      Pool_allocator<MinSize, MaxSize>::memory_requirement(max_blocks));
  return {block, Pool_allocator<MinSize, MaxSize>{block}};
}

class System_allocator {
public:
  // static System_allocator *instance() noexcept { return &_instance; }

  System_allocator() = default;

  Block alloc(Size size) {
    auto const begin = std::malloc(size);
    if (!begin) {
      throw std::bad_alloc{};
    }
    return {static_cast<std::byte *>(begin), size};
  }

  void free(Const_block block) noexcept {
    std::free(const_cast<std::byte *>(block.begin));
  }
};

// class Polymorphic_allocator {
// public:
//   Polymorphic_allocator() = default;

//   template <typename Allocator>
//   Polymorphic_allocator(Allocator *allocator)
//       : _allocator{allocator}, _vtable{&vtable<Allocator>} {}

//   Block alloc(Size size) { return _vtable->alloc(_allocator, size); }

//   void free(Block block) noexcept { _vtable->free(_allocator, block); }

// private:
//   struct Vtable {
//     std::add_pointer_t<Block(void *, Size)> alloc;
//     std::add_pointer_t<void(void *, Block)> free;
//   };

//   template <typename Allocator>
//   static auto constexpr vtable = Vtable{
//       .alloc =
//           [](void *allocator, Size size) {
//             return static_cast<Allocator *>(allocator)->alloc(size);
//           },
//       .free =
//           [](void *allocator, Block block) {
//             return static_cast<Allocator *>(allocator)->free(block);
//           },
//   };

//   void *_allocator{};
//   Vtable *_vtable{};
// };

// class Unique_block {
// public:
//   constexpr Unique_block() noexcept : _block{}, _allocator{} {}

//   Unique_block(Block block, Allocator *allocator)
//       : _block{block}, _allocator{allocator} {}

//   ~Unique_block() {
//     if (_allocator) {
//       _allocator->free(_block);
//     }
//   }

//   Unique_block(Unique_block &&other) noexcept
//       : _block{std::exchange(other._block, Block{})},
//         _allocator{std::exchange(other._allocator, nullptr)} {}

//   Unique_block &operator=(Unique_block &&other) noexcept {
//     auto temp{std::move(other)};
//     swap(temp);
//     return *this;
//   }

//   Block get() const noexcept { return _block; }

//   Block release() noexcept {
//     auto const block = _block;
//     _block = Block{};
//     return block;
//   }

//   void *begin() const noexcept { return _block.begin; }

//   void *end() const noexcept { return _block.end; }

// private:
//   void swap(Unique_block &other) noexcept {
//     std::swap(_block, other._block);
//     std::swap(_allocator, other._allocator);
//   }

//   Block _block;
//   Polymorphic_allocator _allocator;
// };

// inline Unique_block<Allocator> Allocator::alloc_unique(Size size) {
//   return {alloc(size), this};
// }
} // namespace util
} // namespace marlon

#endif