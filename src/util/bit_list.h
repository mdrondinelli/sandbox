#ifndef MARLON_UTIL_BIT_LIST_H
#define MARLON_UTIL_BIT_LIST_H

// #include "list.h"

#include <cstddef>
#include <cstdint>

#include "memory.h"

namespace marlon {
namespace util {
class Bit_list {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t max_size) noexcept {
    return decltype(_impl)::memory_requirement((max_size + 63) / 64);
  }

  constexpr Bit_list() noexcept = default;

  explicit Bit_list(Block block, std::size_t max_size) noexcept
      : Bit_list{block.begin, max_size} {}

  explicit Bit_list(void *block_begin, std::size_t max_size) noexcept
      : _impl{block_begin, (max_size + 63) / 64} {}

  bool empty() {
    return _impl.empty();
  }



private:
  std::uint64_t *_begin;
  std::size_t _max_size;
  std::size_t _size;
};
} // namespace util
} // namespace marlon

#endif