#ifndef MARLON_UTIL_HASH_H
#define MARLON_UTIL_HASH_H

#include <cstddef>
#include <cstdint>

namespace marlon {
namespace util {
template <typename T> struct Hash;

template <> struct Hash<std::int32_t> {
  constexpr std::size_t operator()(std::int32_t x) const noexcept {
    return x * std::int32_t(31);
  }
};

template <> struct Hash<std::uint32_t> {
  constexpr std::size_t operator()(std::uint32_t x) const noexcept {
    return x * std::uint32_t(31);
  }
};

template <> struct Hash<std::nullptr_t> {
  constexpr std::size_t operator()(std::nullptr_t) const noexcept { return 0; }
};

template <typename T> struct Hash<T *> {
  constexpr std::size_t operator()(T *ptr) const noexcept {
    return reinterpret_cast<std::size_t>(ptr) * 31;
  }
};
} // namespace util
} // namespace marlon

#endif