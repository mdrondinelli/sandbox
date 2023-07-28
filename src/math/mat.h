#ifndef MARLON_MATH_MAT_H
#define MARLON_MATH_MAT_H

#include "quat.h"
#include "vec.h"

namespace marlon {
namespace math {
template <typename T, int N, int M> class Mat;

template <typename T, int M> class Mat<T, 2, M> {
public:
  static auto zero() {
    return Mat<T, 2, M>{Vec<T, M>::zero(), Vec<T, M>::zero()};
  }

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
  static auto zero() {
    return Mat<T, 3, M>{Vec<T, M>::zero(), Vec<T, M>::zero(),
                        Vec<T, M>::zero()};
  }

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
  static auto zero() {
    return Mat<T, 4, M>{Vec<T, M>::zero(), Vec<T, M>::zero(), Vec<T, M>::zero(),
                        Vec<T, M>::zero()};
  }

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

template <typename T, int N1, int N2, int N3>
constexpr auto operator*(Mat<T, N1, N2> const &a,
                         Mat<T, N2, N3> const &b) noexcept {
  auto ab = Mat<T, N1, N3>::zero();
  for (auto i = 0; i < N1; ++i) {
    for (auto j = 0; j < N3; ++j) {
      for (auto k = 0; k < N2; ++k) {
        ab[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return ab;
}

template <typename T>
constexpr Mat<T, 3, 4> make_translation_mat3x4(math::Vec3f const &v) noexcept {
  return {{T(1), T(0), T(0), v.x},
          {T(0), T(1), T(0), v.y},
          {T(0), T(0), T(1), v.z}};
}

template <typename T>
constexpr Mat<T, 3, 4> make_translation_mat4x4(math::Vec3f const &v) noexcept {
  return {{T(1), T(0), T(0), v.x},
          {T(0), T(1), T(0), v.y},
          {T(0), T(0), T(1), v.z},
          {T(0), T(0), T(0), T(1)}};
}

template <typename T>
constexpr Mat<T, 3, 3> make_rotation_mat3x3(Quat<T> const &q) noexcept {
  return {{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
           T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
           T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y},
          {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
           T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x},
          {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
           T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y}};
}

template <typename T>
constexpr Mat<T, 3, 4> make_rotation_mat3x4(Quat<T> const &q) noexcept {
  return {{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
           T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
           T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y, T(0)},
          {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
           T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x, T(0)},
          {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
           T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y, T(0)}};
}

template <typename T>
constexpr Mat<T, 4, 4> make_rotation_mat4x4(Quat<T> const &q) noexcept {
  return {{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
           T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
           T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y, T(0)},
          {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
           T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x, T(0)},
          {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
           T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y, T(0)},
          {T(0), T(0), T(0), T(1)}};
}

template <typename T>
constexpr Mat<T, 3, 4>
make_rigid_transform_mat3x4(math::Vec3f const &translation,
                            Quat<T> const &rotation) noexcept {
  auto retval = make_rotation_mat3x4(rotation);
  retval[0][3] = translation.x;
  retval[1][3] = translation.y;
  retval[2][3] = translation.z;
  return retval;
}

template <typename T>
constexpr Mat<T, 4, 4>
make_rigid_transform_mat4x4(math::Vec3f const &translation,
                            Quat<T> const &rotation) noexcept {
  auto retval = make_rotation_mat4x4(rotation);
  retval[0][3] = translation.x;
  retval[1][3] = translation.y;
  retval[2][3] = translation.z;
  return retval;
}

template <typename T>
constexpr Mat<T, 3, 4> rigid_inverse(math::Mat<T, 3, 4> const &m) noexcept {
  auto const translation = math::Vec3f{m[0][3], m[1][3], m[2][3]};
  auto const retval_upper_left = math::Mat3x3f{{m[0][0], m[1][0], m[2][0]},
                                               {m[0][1], m[1][1], m[2][1]},
                                               {m[0][2], m[1][2], m[2][2]}};
  return {{retval_upper_left[0][0], retval_upper_left[0][1],
           retval_upper_left[0][2], -dot(retval_upper_left[0], translation)},
          {retval_upper_left[1][0], retval_upper_left[1][1],
           retval_upper_left[1][2], -dot(retval_upper_left[1], translation)},
          {retval_upper_left[2][0], retval_upper_left[2][1],
           retval_upper_left[2][2], -dot(retval_upper_left[2], translation)}};
}

template <typename T>
constexpr Mat<T, 4, 4> rigid_inverse(math::Mat<T, 4, 4> const &m) noexcept {
  auto const translation = math::Vec3f{m[0][3], m[1][3], m[2][3]};
  auto const retval_upper_left = math::Mat3x3f{{m[0][0], m[1][0], m[2][0]},
                                               {m[0][1], m[1][1], m[2][1]},
                                               {m[0][2], m[1][2], m[2][2]}};
  return {{retval_upper_left[0][0], retval_upper_left[0][1],
           retval_upper_left[0][2], -dot(retval_upper_left[0], translation)},
          {retval_upper_left[1][0], retval_upper_left[1][1],
           retval_upper_left[1][2], -dot(retval_upper_left[1], translation)},
          {retval_upper_left[2][0], retval_upper_left[2][1],
           retval_upper_left[2][2], -dot(retval_upper_left[2], translation)},
          {T(0), T(0), T(0), T(1)}};
}
} // namespace math
} // namespace marlon

#endif