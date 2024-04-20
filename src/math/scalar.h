#ifndef MARLON_MATH_SCALAR_H
#define MARLON_MATH_SCALAR_H

#include <cmath>

namespace marlon {
  namespace math {
    template <typename T>
    constexpr T min(T a, T b) noexcept {
      return a < b ? a : b;
    }

    template <typename T>
    constexpr T max(T a, T b) noexcept {
      return a < b ? b : a;
    }

    template <typename T>
    constexpr T abs(T x) noexcept {
      return std::abs(x);
    }

    template <typename T>
    constexpr T pow(T x, T y) noexcept {
      return std::pow(x, y);
    }
  }
}

#endif