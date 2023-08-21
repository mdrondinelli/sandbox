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

  static auto identity() {
    static_assert(M == 2);
    return Mat<T, 2, 2>{Vec<T, 2>{T(1), T(0)}, Vec<T, 2>{T(0), T(1)}};
  }

  Mat() = default;

  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1) noexcept
      : rows{row0, row1} {}

  template <typename F>
  constexpr explicit Mat(F &&f) noexcept(noexcept(f(0))) : rows{f(0), f(1)} {}

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
  static constexpr auto zero() noexcept {
    return Mat<T, 3, M>{Vec<T, M>::zero(), Vec<T, M>::zero(),
                        Vec<T, M>::zero()};
  }

  static constexpr auto identity() noexcept {
    static_assert(M == 3 || M == 4);
    if constexpr (M == 3) {
      return Mat<T, 3, 3>{Vec<T, 3>{T(1), T(0), T(0)},
                          Vec<T, 3>{T(0), T(1), T(0)},
                          Vec<T, 3>{T(0), T(0), T(1)}};
    } else {
      return Mat<T, 3, 4>{Vec<T, 4>{T(1), T(0), T(0), T(0)},
                          Vec<T, 4>{T(0), T(1), T(0), T(0)},
                          Vec<T, 4>{T(0), T(0), T(1), T(0)}};
    }
  }

  static constexpr auto translation(math::Vec3<T> const &v) noexcept {
    static_assert(M == 4);
    return Mat<T, 3, 4>{{T(1), T(0), T(0), v.x},
                        {T(0), T(1), T(0), v.y},
                        {T(0), T(0), T(1), v.z}};
  }

  static constexpr auto rotation(Quat<T> const &q) noexcept {
    static_assert(M == 3 || M == 4);
    if constexpr (M == 3) {
      return Mat<T, 3, 3>{{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
                           T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
                           T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y},
                          {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
                           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
                           T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x},
                          {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
                           T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
                           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y}};
    } else {
      return Mat<T, 3, 4>{{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
                           T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
                           T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y, T(0)},
                          {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
                           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
                           T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x, T(0)},
                          {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
                           T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
                           T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y,
                           T(0)}};
    }
  }

  static constexpr auto rigid(Vec3f const &t, Quatf const &r) noexcept {
    static_assert(M == 4);
    return Mat<T, 3, 4>{{T(1) - T(2) * r.v.y * r.v.y - T(2) * r.v.z * r.v.z,
                         T(2) * r.v.x * r.v.y - T(2) * r.w * r.v.z,
                         T(2) * r.v.x * r.v.z + T(2) * r.w * r.v.y, t.x},
                        {T(2) * r.v.x * r.v.y + T(2) * r.w * r.v.z,
                         T(1) - T(2) * r.v.x * r.v.x - T(2) * r.v.z * r.v.z,
                         T(2) * r.v.y * r.v.z - T(2) * r.w * r.v.x, t.y},
                        {T(2) * r.v.x * r.v.z - T(2) * r.w * r.v.y,
                         T(2) * r.v.y * r.v.z + T(2) * r.w * r.v.x,
                         T(1) - T(2) * r.v.x * r.v.x - T(2) * r.v.y * r.v.y,
                         t.z}};
  }

  Mat() = default;

  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1,
                Vec<T, M> const &row2) noexcept
      : rows{row0, row1, row2} {}

  constexpr Mat(Mat<T, 2, M> const &row0, Vec<T, M> const &row2) noexcept
      : rows{row0[0], row0[1], row2} {}

  constexpr Mat(Vec<T, M> const &row0, Mat<T, 2, M> const &row1) noexcept
      : rows{row0, row1[0], row1[1]} {}

  template <typename F>
  constexpr explicit Mat(F &&f) noexcept(noexcept(f(0)))
      : rows{f(0), f(1), f(2)} {}

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
  static constexpr auto zero() {
    return Mat<T, 4, M>{Vec<T, M>::zero(), Vec<T, M>::zero(), Vec<T, M>::zero(),
                        Vec<T, M>::zero()};
  }

  static constexpr auto identity() {
    static_assert(M == 4);
    return Mat<T, 4, 4>{
        Vec<T, 4>{T(1), T(0), T(0), T(0)}, Vec<T, 4>{T(0), T(1), T(0), T(0)},
        Vec<T, 4>{T(0), T(0), T(1), T(0)}, Vec<T, 4>{T(0), T(0), T(0), T(1)}};
  }

  static constexpr auto translation(math::Vec3<T> const &v) noexcept {
    static_assert(M == 4);
    return Mat<T, 4, 4>{{T(1), T(0), T(0), v.x},
                        {T(0), T(1), T(0), v.y},
                        {T(0), T(0), T(1), v.z},
                        {T(0), T(0), T(0), T(1)}};
  }

  static constexpr auto rotation(Quat<T> const &q) noexcept {
    static_assert(M == 4);
    return Mat<T, 4, 4>{{T(1) - T(2) * q.v.y * q.v.y - T(2) * q.v.z * q.v.z,
                         T(2) * q.v.x * q.v.y - T(2) * q.w * q.v.z,
                         T(2) * q.v.x * q.v.z + T(2) * q.w * q.v.y, T(0)},
                        {T(2) * q.v.x * q.v.y + T(2) * q.w * q.v.z,
                         T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.z * q.v.z,
                         T(2) * q.v.y * q.v.z - T(2) * q.w * q.v.x, T(0)},
                        {T(2) * q.v.x * q.v.z - T(2) * q.w * q.v.y,
                         T(2) * q.v.y * q.v.z + T(2) * q.w * q.v.x,
                         T(1) - T(2) * q.v.x * q.v.x - T(2) * q.v.y * q.v.y,
                         T(0)},
                        {T(0), T(0), T(0), T(1)}};
  }

  static constexpr auto rigid(Vec3f const &t, Quatf const &r) noexcept {
    static_assert(M == 4);
    return Mat<T, 4, 4>{{T(1) - T(2) * r.v.y * r.v.y - T(2) * r.v.z * r.v.z,
                         T(2) * r.v.x * r.v.y - T(2) * r.w * r.v.z,
                         T(2) * r.v.x * r.v.z + T(2) * r.w * r.v.y, t.x},
                        {T(2) * r.v.x * r.v.y + T(2) * r.w * r.v.z,
                         T(1) - T(2) * r.v.x * r.v.x - T(2) * r.v.z * r.v.z,
                         T(2) * r.v.y * r.v.z - T(2) * r.w * r.v.x, t.y},
                        {T(2) * r.v.x * r.v.z - T(2) * r.w * r.v.y,
                         T(2) * r.v.y * r.v.z + T(2) * r.w * r.v.x,
                         T(1) - T(2) * r.v.x * r.v.x - T(2) * r.v.y * r.v.y,
                         t.z},
                        {T(0), T(0), T(0), T(1)}};
  }

  Mat() = default;

  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1,
                Vec<T, M> const &row2, Vec<T, M> const &row3) noexcept
      : rows{row0, row1, row2, row3} {}

  constexpr Mat(Mat<T, 2, M> const &row0, Vec<T, M> const &row2,
                Vec<T, M> const &row3) noexcept
      : rows{row0[0], row0[1], row2, row3} {}

  constexpr Mat(Vec<T, M> const &row0, Mat<T, 2, M> const &row1,
                Vec<T, M> const &row3) noexcept
      : rows{row0, row1[0], row1[1], row3} {}

  constexpr Mat(Vec<T, M> const &row0, Vec<T, M> const &row1,
                Mat<T, 2, M> const &row2) noexcept
      : rows{row0, row1, row2[0], row2[1]} {}

  constexpr Mat(Mat<T, 3, M> const &row0, Vec<T, M> const &row3) noexcept
      : rows{row0[0], row0[1], row0[2], row3} {}

  constexpr Mat(Vec<T, M> const &row0, Mat<T, 3, M> const &row1) noexcept
      : rows{row0, row1[0], row1[1], row1[2]} {}

  template <typename F>
  constexpr explicit Mat(F &&f) noexcept(noexcept(f(0)))
      : rows{f(0), f(1), f(2), f(3)} {}

  template <typename U>
  constexpr explicit Mat(Mat<U, 4, M> const &m) noexcept
      : rows{static_cast<Vec<T, M>>(m[0]), static_cast<Vec<T, M>>(m[1]),
             static_cast<Vec<T, M>>(m[2]), static_cast<Vec<T, M>>(m[3])} {}

  constexpr auto const &operator[](int n) const noexcept { return rows[n]; }

  constexpr auto &operator[](int n) noexcept { return rows[n]; }

private:
  Vec<T, M> rows[4];
};

template <typename T> using Mat2x2 = Mat<T, 2, 2>;
template <typename T> using Mat2x3 = Mat<T, 2, 3>;
template <typename T> using Mat2x4 = Mat<T, 2, 4>;
template <typename T> using Mat3x2 = Mat<T, 3, 2>;
template <typename T> using Mat3x3 = Mat<T, 3, 3>;
template <typename T> using Mat3x4 = Mat<T, 3, 4>;
template <typename T> using Mat4x2 = Mat<T, 4, 2>;
template <typename T> using Mat4x3 = Mat<T, 4, 3>;
template <typename T> using Mat4x4 = Mat<T, 4, 4>;
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

template <typename T, int N, int M>
constexpr Mat<T, N, M> operator*(T s, Mat<T, N, M> const &m) noexcept {
  return Mat<T, N, M>{[&](int i) { return s * m[i]; }};
}

template <typename T, int N, int M>
constexpr Mat<T, N, M> operator*(Mat<T, N, M> const &m, T s) noexcept {
  return Mat<T, N, M>{[&](int i) { return m[i] * s; }};
}

template <typename T, int N, int M>
constexpr Mat<T, N, M> operator/(Mat<T, N, M> const &m, T s) noexcept {
  if constexpr (std::is_floating_point_v<T>) {
    return m * (1 / s);
  } else {
    return Mat<T, N, M>{[&](int i) { return m[i] / s; }};
  };
}

template <typename T, int N1, int N2, int N3>
constexpr Mat<T, N1, N3> operator*(Mat<T, N1, N2> const &a,
                                   Mat<T, N2, N3> const &b) noexcept {
  auto retval = Mat<T, N1, N3>::zero();
  for (auto i = 0; i < N1; ++i) {
    for (auto j = 0; j < N3; ++j) {
      for (auto k = 0; k < N2; ++k) {
        retval[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return retval;
}

template <typename T, int N, int M>
constexpr Mat<T, N, M> &operator*=(Mat<T, N, M> &a,
                                   Mat<T, M, M> const &b) noexcept {
  return a = a * b;
}

template <typename T, int N, int M>
constexpr Mat<T, N, M> &operator/=(Mat<T, N, M> &m, T s) noexcept {
  return m = m / s;
}

template <typename T, int N, int M>
constexpr Mat<T, M, N> transpose(math::Mat<T, N, M> const &m) noexcept {
  auto retval = Mat<T, M, N>::zero();
  for (auto i = 0; i < M; ++i) {
    for (auto j = 0; j < N; ++j) {
      retval[i][j] = m[j][i];
    }
  }
  return retval;
}

template <typename T>
constexpr T determinant(math::Mat<T, 2, 2> const &m) noexcept {
  return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}

template <typename T>
constexpr T determinant(math::Mat<T, 3, 3> const &m) noexcept {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

template <typename T>
constexpr T determinant(math::Mat<T, 4, 4> const &m) noexcept {
  return m[0][0] * (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) +
                    m[1][2] * (m[2][3] * m[3][1] - m[2][1] * m[3][3]) +
                    m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) -
         m[0][1] * (m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) +
                    m[1][2] * (m[2][3] * m[3][0] - m[2][0] * m[3][3]) +
                    m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) +
         m[0][2] * (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
                    m[1][1] * (m[2][3] * m[3][0] - m[2][0] * m[3][3]) +
                    m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) -
         m[0][3] * (m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) +
                    m[1][1] * (m[2][2] * m[3][0] - m[2][0] * m[3][2]) +
                    m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
}

template <typename T>
constexpr Mat<T, 2, 2> cofactors(math::Mat<T, 2, 2> const &m) noexcept {
  return {{m[1][1], -m[1][0]}, {-m[0][1], m[0][0]}};
}

template <typename T>
constexpr Mat<T, 3, 3> cofactors(math::Mat<T, 3, 3> const &m) noexcept {
  return {{determinant(Mat<T, 2, 2>{{m[1][1], m[1][2]}, {m[2][1], m[2][2]}}),
           -determinant(Mat<T, 2, 2>{{m[1][0], m[1][2]}, {m[2][0], m[2][2]}}),
           determinant(Mat<T, 2, 2>{{m[1][0], m[1][1]}, {m[2][0], m[2][1]}})},
          {-determinant(Mat<T, 2, 2>{{m[0][1], m[0][2]}, {m[2][1], m[2][2]}}),
           determinant(Mat<T, 2, 2>{{m[0][0], m[0][2]}, {m[2][0], m[2][2]}}),
           -determinant(Mat<T, 2, 2>{{m[0][0], m[0][1]}, {m[2][0], m[2][1]}})},
          {determinant(Mat<T, 2, 2>{{m[0][1], m[0][2]}, {m[1][1], m[1][2]}}),
           -determinant(Mat<T, 2, 2>{{m[0][0], m[0][2]}, {m[1][0], m[1][2]}}),
           determinant(Mat<T, 2, 2>{{m[0][0], m[0][1]}, {m[1][0], m[1][1]}})}};
}

template <typename T, int N>
constexpr Mat<T, N, N> adjoint(math::Mat<T, N, N> const &m) noexcept {
  return transpose(cofactors(m));
}

template <typename T, int N>
constexpr Mat<T, N, N> inverse(math::Mat<T, N, N> const &m) noexcept {
  return adjoint(m) / determinant(m);
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