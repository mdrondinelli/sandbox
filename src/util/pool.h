#ifndef MARLON_UTIL_POOL_H
#define MARLON_UTIL_POOL_H

#include <algorithm>

#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Pool {
public:
  template <typename Allocator>
  static std::pair<Block, Pool> make(Allocator &allocator, Size max_objects) {
    auto const block = allocator.alloc(memory_requirement(max_objects));
    return {block, Pool{block, max_objects}};
  }

  static constexpr Size memory_requirement(Size max_objects) noexcept {
    return sizeof(T) * max_objects;
  }

  Pool() noexcept = default;

  Pool(Pool<T> const &other) = delete;

  Pool &operator=(Pool<T> const &other) = delete;

  Pool(Pool<T> &&other) = default;

  Pool &operator=(Pool<T> &&other) = default;

  explicit Pool(Block block, Size max_objects) noexcept
      : Pool{block.begin, max_objects} {}

  explicit Pool(std::byte *block_begin, Size max_objects) noexcept
      : _allocator{Stack_allocator<alignof(T)>{
            {block_begin, memory_requirement(max_objects)}}} {}

  template <typename... Args> T *emplace(Args &&...args) {
    return new (_allocator.alloc(sizeof(T)).begin)
        T(std::forward<Args>(args)...);
  }

  void erase(T *object) {
    object->~T();
    _allocator.free({reinterpret_cast<std::byte *>(object), sizeof(T)});
  }

private:
  Free_list_allocator<Stack_allocator<alignof(T)>,
                      sizeof(T),
                      std::max(sizeof(T), sizeof(void *))>
      _allocator;
};
} // namespace util
} // namespace marlon

#endif