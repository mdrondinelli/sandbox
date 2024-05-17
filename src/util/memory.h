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
struct Block {
  void *begin;
  void *end;
};

constexpr Block make_block(void *begin, void *end) noexcept {
  return {begin, end};
}

constexpr Block make_block(void *begin, Size size) noexcept {
  return {begin, static_cast<std::byte *>(begin) + size};
}

constexpr Size align(Size size, Size alignment) noexcept {
  return (size + alignment - 1) & -alignment;
}

inline Size ptrdiff(void const *p1, void const *p2) noexcept {
  return std::bit_cast<std::uintptr_t>(p1) - std::bit_cast<std::uintptr_t>(p2);
}

class Unique_block;

// class Allocator {
// public:
//   virtual ~Allocator() = default;

//   virtual Block alloc(Size size) = 0;

//   virtual void free(Block block) noexcept = 0;

//   Unique_block alloc_unique(Size size);
// };

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

  Block block() const noexcept { return _block; }

  Block alloc(Size size) {
    auto const block_end = static_cast<std::byte *>(_top) + size;
    auto const aligned_block_end =
        static_cast<std::byte *>(_top) + align(size, Alignment);
    if (aligned_block_end <= _block.end) {
      auto const result = Block{.begin = _top, .end = block_end};
      _top = aligned_block_end;
      return result;
    } else {
      throw std::bad_alloc{};
    }
  }

  void free(Block block) noexcept {
    auto const block_size = static_cast<std::byte *>(block.end) -
                            static_cast<std::byte *>(block.begin);
    auto const aligned_block_size = align(block_size, Alignment);
    auto const aligned_block_end =
        static_cast<std::byte *>(block.begin) + aligned_block_size;
    if (aligned_block_end == _top) {
      _top = block.begin;
    }
  }

  bool owns(Block const block) const noexcept {
    return block.begin >= _block.begin && block.begin < _block.end;
  }

private:
  Block _block;
  void *_top;
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
        auto const result = make_block(_root, size);
        _root = _root->next;
        return result;
      } else {
        return make_block(_parent.alloc(MaxSize).begin, size);
      }
    } else {
      return _parent.alloc(size);
    }
  }

  void free(Block block) noexcept {
    auto const size = ptrdiff(block.end, block.begin);
    if (size >= MinSize && size <= MaxSize) {
      _root = new (block.begin) Node{.next = _root};
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
  static constexpr Size memory_requirement(Size max_blocks) {
    return MaxSize * max_blocks;
  }

  Pool_allocator() noexcept = default;

  explicit Pool_allocator(Block block) noexcept
      : _impl{Stack_allocator<1>{block}} {}

  Block alloc(Size size) { return _impl.alloc(size); }

  void free(Block block) noexcept { _impl.free(block); }

  Block block() const noexcept { return _impl.parent().block(); }

  Size max_blocks() const noexcept {
    auto const blk = block();
    return ptrdiff(blk.end, blk.begin) / MaxSize;
  }

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
    return make_block(begin, size);
  }

  void free(Block block) noexcept { std::free(block.begin); }
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