#ifndef MARLON_MATH_SCALAR_H
#define MARLON_MATH_SCALAR_H

#include <cmath>

namespace marlon {
  namespace math {
    template <typename T>
    constexpr T const &min(T const &a, T const &b) noexcept {
      return a < b ? a : b;
    }

    template <typename T>
    constexpr T const &max(T const &a, T const &b) noexcept {
      return a < b ? b : a;
    }

    template <typename T>
    constexpr T const &clamp(T const &v, T const &lo, T const &hi) noexcept {
      return min(max(v, lo), hi);
    }

    template <typename T>
    constexpr T abs(T x) noexcept {
      return std::abs(x);
    }

    template <typename T>
    constexpr T pow(T x, T y) noexcept {
      return std::pow(x, y);
    }

    template <typename T>
    constexpr T sqrt(T x) noexcept {
      return std::sqrt(x);
    }
  }
}

#endif