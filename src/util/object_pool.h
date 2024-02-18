#ifndef MARLON_UTIL_OBJECT_POOL_H
#define MARLON_UTIL_OBJECT_POOL_H

#include <algorithm>

#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Pool {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t max_objects) noexcept {
    return sizeof(T) * max_objects;
  }

  Pool() noexcept = default;

  Pool(Pool<T> const &other) = delete;

  Pool &operator=(Pool<T> const &other) = delete;

  Pool(Pool<T> &&other) = default;

  Pool &operator=(Pool<T> &&other) = default;

  explicit Pool(Block block, std::size_t max_objects) noexcept
      : Pool{block.begin, max_objects} {}

  explicit Pool(void *block_begin, std::size_t max_objects) noexcept
      : _allocator{Stack_allocator<alignof(T)>{
            make_block(block_begin, memory_requirement(max_objects))}} {}

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

template <typename T, typename Allocator>
std::pair<Block, Pool<T>> make_pool(Allocator &allocator,
                                    std::size_t max_objects) {
  auto const block = allocator.alloc(Pool<T>::memory_requirement(max_objects));
  return {block, Pool<T>(block, max_objects)};
}
} // namespace util
} // namespace marlon

#endif