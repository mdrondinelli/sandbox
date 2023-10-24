#ifndef MARLON_UTIL_MEMORY_H
#define MARLON_UTIL_MEMORY_H

#include <cstddef>
#include <cstdlib>

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

constexpr std::size_t align(std::size_t size) noexcept {
  return (size + 15) & static_cast<std::size_t>(-16);
}

class Stack_allocator {
public:
  static constexpr std::size_t memory_requirement(
      std::initializer_list<std::size_t> allocation_sizes) noexcept {
    auto result = std::size_t{};
    for (auto const size : allocation_sizes) {
      result += align(size);
    }
    return result;
  }

  explicit Stack_allocator(Block block) noexcept
      : _block{block}, _top{block.begin} {}

  Block allocate(std::size_t const size) {
    auto const block_end = static_cast<std::byte *>(_top) + size;
    auto const aligned_block_end = static_cast<std::byte *>(_top) + align(size);
    if (aligned_block_end <= _block.end) {
      auto const result = Block{.begin = _top, .end = block_end};
      _top = aligned_block_end;
      return result;
    } else {
      throw std::bad_alloc{};
    }
  }

  void deallocate(Block const block) noexcept {
    auto const block_size = static_cast<std::byte *>(block.end) -
                            static_cast<std::byte *>(block.begin);
    auto const aligned_block_size = align(block_size);
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

template <class Parent, std::size_t Size> class Free_list_allocator {
public:
  Free_list_allocator() = default;

  explicit Free_list_allocator(Parent const &parent) : _parent{parent} {}

  explicit Free_list_allocator(Parent &&parent) : _parent{std::move(parent)} {}

  Block allocate(std::size_t size) {
    if (size == Size && _root) {
      auto const result = Block{.begin = _root, _root + size};
      _root = _root->next;
      return result;
    }
    return _parent.allocate(size);
  }

  void free(Block block) noexcept {
    auto const size = static_cast<std::byte *>(block.end) -
                      static_cast<std::byte *>(block.begin);
    if (size == Size) {
      _root = new (block.begin) Node{.next = _root};
    } else {
      _parent.deallocate(block);
    }
  }

private:
  struct Node {
    Node *next;
  };

  Parent _parent;
  Node *_root{};

  static_assert(Size >= sizeof(Node));
};
} // namespace util
} // namespace marlon

#endif