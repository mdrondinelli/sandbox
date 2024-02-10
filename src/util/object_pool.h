#ifndef MARLON_UTIL_OBJECT_POOL_H
#define MARLON_UTIL_OBJECT_POOL_H

#include <algorithm>

#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Object_pool {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t capacity) noexcept {
    return sizeof(T) * capacity;
  }

  Object_pool() noexcept = default;

  Object_pool(Object_pool<T> const &other) = delete;

  Object_pool &operator=(Object_pool<T> const &other) = delete;

  Object_pool(Object_pool<T> &&other) = default;

  Object_pool &operator=(Object_pool<T> &&other) = default;

  explicit Object_pool(Block block, std::size_t capacity) noexcept
      : Object_pool{block.begin, capacity} {}

  explicit Object_pool(void *block_begin, std::size_t capacity) noexcept
      : _allocator{Stack_allocator<alignof(T)>{
            make_block(block_begin, memory_requirement(capacity))}} {}

  template <typename... Args> T *alloc(Args &&...args) {
    return new (_allocator.alloc(sizeof(T)).begin)
        T(std::forward<Args>(args)...);
  }

  void free(T *object) {
    object->~T();
    _allocator.free(make_block(object, sizeof(T)));
  }

private:
  Free_list_allocator<Stack_allocator<alignof(T)>, sizeof(T),
                      std::max(sizeof(T), sizeof(void *))>
      _allocator;
};
} // namespace util
} // namespace marlon

#endif