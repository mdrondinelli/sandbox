#ifndef MARLON_MATH_MAT_H
#define MARLON_MATH_MAT_H

#include "vec.h"

namespace marlon {
namespace math {
template <typename T, int N, int M> class Mat;

template <typename T, int M> class Mat<T, 2, M> {
public:
  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1) noexcept
      : rows{row0, row1} {}

  template <typename U>
  constexpr explicit Mat(Mat<U, 2, M> const &m) noexcept
      : rows{static_cast<Vec<T, M>>(m[0]), static_cast<Vec<T, M>>(m[1])} {}

  constexpr auto const &operator[](int n) const noexcept { return rows[n]; }

  constexpr auto &operator[](int n) noexcept { return rows[n]; }

private:
  Vec<T, M> rows[2];
};

template <typename T, int M> class Mat<T, 3, M> {
public:
  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1,
                Vec<T, M> const &row2) noexcept
      : rows{row0, row1, row2} {}

  template <typename U>
  constexpr explicit Mat(Mat<U, 3, M> const &m) noexcept
      : rows{static_cast<Vec<T, M>>(m[0]), static_cast<Vec<T, M>>(m[1]),
             static_cast<Vec<T, M>>(m[2])} {}

  constexpr auto const &operator[](int n) const noexcept { return rows[n]; }

  constexpr auto &operator[](int n) noexcept { return rows[n]; }

private:
  Vec<T, M> rows[3];
};

template <typename T, int M> class Mat<T, 4, M> {
public:
  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1,
                Vec<T, M> const &row2, Vec<T, M> const &row3) noexcept
      : rows{row0, row1, row2, row3} {}

  template <typename U>
  constexpr explicit Mat(Mat<U, 4, M> const &m) noexcept
      : rows{static_cast<Vec<T, M>>(m[0]), static_cast<Vec<T, M>>(m[1]),
             static_cast<Vec<T, M>>(m[2]), static_cast<Vec<T, M>>(m[3])} {}

  constexpr auto const &operator[](int n) const noexcept { return rows[n]; }

  constexpr auto &operator[](int n) noexcept { return rows[n]; }

private:
  Vec<T, M> rows[4];
};

using Mat2x2i = Mat<std::int32_t, 2, 2>;
using Mat2x3i = Mat<std::int32_t, 2, 3>;
using Mat2x4i = Mat<std::int32_t, 2, 4>;
using Mat3x2i = Mat<std::int32_t, 3, 2>;
using Mat3x3i = Mat<std::int32_t, 3, 3>;
using Mat3x4i = Mat<std::int32_t, 3, 4>;
using Mat4x2i = Mat<std::int32_t, 4, 2>;
using Mat4x3i = Mat<std::int32_t, 4, 3>;
using Mat4x4i = Mat<std::int32_t, 4, 4>;
using Mat2x2f = Mat<float, 2, 2>;
using Mat2x3f = Mat<float, 2, 3>;
using Mat2x4f = Mat<float, 2, 4>;
using Mat3x2f = Mat<float, 3, 2>;
using Mat3x3f = Mat<float, 3, 3>;
using Mat3x4f = Mat<float, 3, 4>;
using Mat4x2f = Mat<float, 4, 2>;
using Mat4x3f = Mat<float, 4, 3>;
using Mat4x4f = Mat<float, 4, 4>;
using Mat2x2d = Mat<double, 2, 2>;
using Mat2x3d = Mat<double, 2, 3>;
using Mat2x4d = Mat<double, 2, 4>;
using Mat3x2d = Mat<double, 3, 2>;
using Mat3x3d = Mat<double, 3, 3>;
using Mat3x4d = Mat<double, 3, 4>;
using Mat4x2d = Mat<double, 4, 2>;
using Mat4x3d = Mat<double, 4, 3>;
using Mat4x4d = Mat<double, 4, 4>;

template <typename T, int N, int M>
constexpr bool operator==(Mat<T, N, M> const &a,
                          Mat<T, N, M> const &b) noexcept {
  for (int i = 0; i < N; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}
} // namespace math
} // namespace marlon

#endif