#ifndef MARLON_UTIL_MEMORY_H
#define MARLON_UTIL_MEMORY_H

#include <cstddef>
#include <cstdlib>

#include <functional>
#include <new>
#include <span>
#include <utility>

namespace marlon {
namespace util {
struct Block {
  void *begin;
  void *end;
};

constexpr Block make_block(void *begin, void *end) noexcept {
  return {begin, end};
}

constexpr Block make_block(void *begin, std::size_t size) noexcept {
  return {begin, static_cast<std::byte *>(begin) + size};
}

constexpr std::size_t align(std::size_t size, std::size_t alignment) noexcept {
  return (size + alignment - 1) & -alignment;
}

inline std::ptrdiff_t ptrdiff(void const *p1, void const *p2) noexcept {
  return reinterpret_cast<std::uintptr_t>(p1) -
         reinterpret_cast<std::uintptr_t>(p2);
}

class Allocator {
public:
  virtual ~Allocator() = default;

  virtual Block alloc(std::size_t size) = 0;

  virtual void free(Block block) noexcept = 0;
};

template <std::size_t Alignment = alignof(std::max_align_t)>
class Stack_allocator : public Allocator {
public:
  static constexpr std::size_t memory_requirement(
      std::initializer_list<std::size_t> allocation_sizes) noexcept {
    auto result = std::size_t{};
    for (auto const size : allocation_sizes) {
      result += align(size, Alignment);
    }
    return result;
  }

  explicit Stack_allocator(Block block) noexcept
      : _block{block}, _top{block.begin} {}

  Block alloc(std::size_t size) final {
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

  void free(Block block) noexcept final {
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

template <class Parent, std::size_t MinSize, std::size_t MaxSize>
class Free_list_allocator : public Allocator {
public:
  Free_list_allocator() = default;

  explicit Free_list_allocator(Parent const &parent) : _parent{parent} {}

  explicit Free_list_allocator(Parent &&parent) : _parent{std::move(parent)} {}

  Block alloc(std::size_t size) final {
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

  void free(Block block) noexcept final {
    auto const size = static_cast<std::size_t>(ptrdiff(block.end, block.begin));
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

class System_allocator : public Allocator {
public:
  static System_allocator *instance() noexcept { return &_instance; }

  Block alloc(std::size_t size) final {
    auto const begin = std::malloc(size);
    if (!begin) {
      throw std::bad_alloc{};
    }
    return make_block(begin, size);
  }

  void free(Block block) noexcept final { std::free(block.begin); }

private:
  System_allocator() = default;

  static System_allocator _instance;
};

class Unique_block {
public:
  Unique_block(Block block, Allocator *allocator)
      : _block{block}, _allocator{allocator} {}

  ~Unique_block() {
    if (_allocator) {
      _allocator->free(_block);
    }
  }

  Unique_block(Unique_block &&other) noexcept
      : _block{std::exchange(other._block, Block{})},
        _allocator{std::exchange(other._allocator, nullptr)} {}

  Block get() const noexcept { return _block; }

  Block release() noexcept {
    auto const block = _block;
    _block = Block{};
    return block;
  }

  void *begin() const noexcept { return _block.begin; }

  void *end() const noexcept { return _block.end; }

private:
  Block _block;
  Allocator *_allocator;
};

inline Unique_block
make_unique_block(std::size_t size,
                  Allocator *allocator = System_allocator::instance()) {
  return {allocator->alloc(size), allocator};
}
} // namespace util
} // namespace marlon

#endif