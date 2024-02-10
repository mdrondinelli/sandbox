#ifndef MARLON_UTIL_EQUAL_H
#define MARLON_UTIL_EQUAL_H

namespace marlon {
namespace util {
template <typename T> struct Equal {
  template <typename U>
  constexpr bool operator()(T const &lhs, U const &rhs) const noexcept {
    return lhs == rhs;
  }
};
} // namespace util
} // namespace marlon

#endif