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

  static auto zero() { return Vec<T, 2>{T(0), T(0)}; }

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

  static auto zero() { return Vec<T, 3>{T(0), T(0), T(0)}; }

  Vec() = default;

  constexpr Vec(T x, T y, T z) : x{x}, y{y}, z{z} {}

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

  Vec() = default;

  constexpr Vec(T x, T y, T z, T w) : x{x}, y{y}, z{z}, w{w} {}

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

using Vec2i = Vec<std::int32_t, 2>;
using Vec3i = Vec<std::int32_t, 3>;
using Vec4i = Vec<std::int32_t, 4>;
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Vec2d = Vec<double, 2>;
using Vec3d = Vec<double, 3>;
using Vec4d = Vec<double, 4>;

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