#ifndef MARLON_MATH_VEC_H
#define MARLON_MATH_VEC_H

#include <cassert>
#include <cmath>
#include <cstdint>

#include <type_traits>

#include "unreachable.h"

namespace marlon {
namespace math {
static_assert(sizeof(float) == 4);
static_assert(sizeof(double) == 8);

template <typename T, int N> struct Vec;

template <typename T> struct Vec<T, 2> {
  T x;
  T y;

  static constexpr auto zero() noexcept { return Vec<T, 2>{T(0), T(0)}; }

  static constexpr auto all(T s) noexcept { return Vec<T, 2>{s, s}; }

  Vec() = default;

  constexpr Vec(T x, T y) : x{x}, y{y} {}

  template <typename F>
  constexpr explicit Vec(F &&f) noexcept(noexcept(f(0))) : x{f(0)}, y{f(1)} {}

  template <typename U>
  constexpr explicit Vec(Vec<U, 2> const &v) noexcept
      : x{static_cast<T>(v.x)}, y{static_cast<T>(v.y)} {}

  constexpr T const &operator[](int n) const noexcept {
    assert(n >= 0 && n < 2);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      unreachable();
    }
  }

  constexpr T &operator[](int n) noexcept {
    assert(n >= 0 && n < 2);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      unreachable();
    }
  }
};

template <typename T> struct Vec<T, 3> {
  T x;
  T y;
  T z;

  static constexpr auto zero() noexcept { return Vec<T, 3>{T(0), T(0), T(0)}; }

  static constexpr auto all(T s) noexcept { return Vec<T, 3>{s, s, s}; }

  Vec() = default;

  constexpr Vec(T x, T y, T z) noexcept : x{x}, y{y}, z{z} {}

  constexpr Vec(Vec<T, 2> xy, T z) noexcept : x{xy.x}, y{xy.y}, z{z} {}

  constexpr Vec(T x, Vec<T, 2> yz) noexcept : x{x}, y{yz.x}, z{yz.y} {}

  template <typename F>
  constexpr explicit Vec(F &&f) noexcept(noexcept(f(0)))
      : x{f(0)}, y{f(1)}, z{f(2)} {}

  template <typename U>
  constexpr explicit Vec(Vec<U, 3> const &v) noexcept
      : x{static_cast<T>(v.x)}, y{static_cast<T>(v.y)}, z{static_cast<T>(v.z)} {
  }

  constexpr T const &operator[](int n) const noexcept {
    assert(n >= 0 && n < 3);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      unreachable();
    }
  }

  constexpr T &operator[](int n) noexcept {
    assert(n >= 0 && n < 3);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      unreachable();
    }
  }
};

template <typename T> struct Vec<T, 4> {
  T x;
  T y;
  T z;
  T w;

  static auto zero() { return Vec<T, 4>{T(0), T(0), T(0), T(0)}; }

  static auto all(T s) { return Vec<T, 4>{s, s, s, s}; }

  Vec() = default;

  constexpr Vec(T x, T y, T z, T w) noexcept : x{x}, y{y}, z{z}, w{w} {}

  constexpr Vec(Vec<T, 2> xy, Vec<T, 2> zw) noexcept
      : x{xy.x}, y{xy.y}, z{zw.x}, w{zw.y} {}

  constexpr Vec(Vec<T, 2> xy, T z, T w) noexcept
      : x{xy.x}, y{xy.y}, z{z}, w{w} {}

  constexpr Vec(T x, Vec<T, 2> yz, T w) noexcept
      : x{x}, y{yz.x}, z{yz.y}, w{w} {}

  constexpr Vec(T x, T y, Vec<T, 2> zw) noexcept
      : x{x}, y{y}, z{zw.x}, w{zw.y} {}

  constexpr Vec(Vec<T, 3> xyz, T w) noexcept
      : x{xyz.x}, y{xyz.y}, z{xyz.z}, w{w} {}

  constexpr Vec(T x, Vec<T, 3> yzw) noexcept
      : x{x}, y{yzw.x}, z{yzw.y}, w{yzw.z} {}

  template <typename F>
  constexpr explicit Vec(F &&f) noexcept(noexcept(f(0)))
      : x{f(0)}, y{f(1)}, z{f(2)}, w{f(3)} {}

  template <typename U>
  constexpr explicit Vec(Vec<U, 4> const &v) noexcept
      : x{static_cast<T>(v.x)}, y{static_cast<T>(v.y)}, z{static_cast<T>(v.z)},
        w{static_cast<T>(v.w)} {}

  constexpr T const &operator[](int n) const noexcept {
    assert(n >= 0 && n < 4);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return w;
    default:
      unreachable();
    }
  }

  constexpr T &operator[](int n) noexcept {
    assert(n >= 0 && n < 4);
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return w;
    default:
      unreachable();
    }
  }
};

template <typename T> using Vec2 = Vec<T, 2>;
template <typename T> using Vec3 = Vec<T, 3>;
template <typename T> using Vec4 = Vec<T, 4>;
using Vec2i = Vec2<std::int32_t>;
using Vec3i = Vec3<std::int32_t>;
using Vec4i = Vec4<std::int32_t>;
using Vec2f = Vec2<float>;
using Vec3f = Vec3<float>;
using Vec4f = Vec4<float>;
using Vec2d = Vec2<double>;
using Vec3d = Vec3<double>;
using Vec4d = Vec4<double>;

template <typename T, int N>
constexpr bool operator==(Vec<T, N> const &u, Vec<T, N> const &v) noexcept {
  for (int i = 0; i < N; ++i) {
    if (u[i] != v[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, int N>
constexpr auto operator+(Vec<T, N> const &v) noexcept {
  return v;
}

template <typename T, int N>
constexpr auto operator-(Vec<T, N> const &v) noexcept {
  return Vec<T, N>{[&](int i) { return -v[i]; }};
}

template <typename T, int N>
constexpr auto operator+(Vec<T, N> const &u, Vec<T, N> const &v) noexcept {
  return Vec<T, N>{[&](int i) { return u[i] + v[i]; }};
}

template <typename T, int N>
constexpr auto operator-(Vec<T, N> const &u, Vec<T, N> const &v) noexcept {
  return Vec<T, N>{[&](int i) { return u[i] - v[i]; }};
}

template <typename T, int N>
constexpr auto operator*(T s, Vec<T, N> const &v) noexcept {
  return Vec<T, N>{[&](int i) { return s * v[i]; }};
}

template <typename T, int N>
constexpr auto operator*(Vec<T, N> const &v, T s) noexcept {
  return Vec<T, N>{[&](int i) { return v[i] * s; }};
}

template <typename T, int N>
constexpr auto operator/(Vec<T, N> const &v, T s) noexcept {
  if constexpr (std::is_floating_point_v<T>) {
    return v * (1 / s);
  } else {
    return Vec<T, N>{[&](int i) { return v[i] / s; }};
  }
}

template <typename T, int N>
constexpr auto &operator+=(Vec<T, N> &u, Vec<T, N> const &v) noexcept {
  return u = (u + v);
}

template <typename T, int N>
constexpr auto &operator-=(Vec<T, N> &u, Vec<T, N> const &v) noexcept {
  return u = (u - v);
}

template <typename T, int N>
constexpr auto &operator*=(Vec<T, N> &v, T s) noexcept {
  return v = (v * s);
}

template <typename T, int N>
constexpr auto &operator/=(Vec<T, N> &v, T s) noexcept {
  return v = (v / s);
}

template <typename T, int N>
constexpr auto length2(Vec<T, N> const &v) noexcept {
  auto retval = T(0);
  for (int i = 0; i < N; ++i) {
    retval += v[i] * v[i];
  }
  return retval;
}

template <typename T, int N>
constexpr auto length(Vec<T, N> const &v) noexcept {
  return std::sqrt(length2(v));
}

template <typename T, int N>
constexpr auto normalize(Vec<T, N> const &v) noexcept {
  return v / length(v);
}

template <typename T, int N>
constexpr auto dot(Vec<T, N> const &a, Vec<T, N> const &b) noexcept {
  auto retval = T(0);
  for (int i = 0; i < N; ++i) {
    retval += a[i] * b[i];
  }
  return retval;
}

template <typename T>
constexpr auto cross(Vec<T, 3> const &a, Vec<T, 3> const &b) noexcept {
  return Vec<T, 3>{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x};
}
} // namespace math
} // namespace marlon

#endif