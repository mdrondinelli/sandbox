#ifndef MARLON_MATH_AABB_BOUNDS_H
#define MARLON_MATH_AABB_BOUNDS_H

#include "vec.h"

namespace marlon::math {
template <typename T, int N> struct Aabb {
  Vec<T, N> min;
  Vec<T, N> max;
};

using Aabb2i = Aabb<std::int32_t, 2>;
using Aabb3i = Aabb<std::int32_t, 3>;
using Aabb2f = Aabb<float, 2>;
using Aabb3f = Aabb<float, 3>;
using Aabb2d = Aabb<double, 2>;
using Aabb3d = Aabb<double, 3>;

template <typename T, int N>
inline Vec<T, N> center(Aabb<T, N> const &box) noexcept {
  return (box.min + box.max) / T(2);
}

template <typename T, int N>
inline Vec<T, N> extents(Aabb<T, N> const &box) noexcept {
  return box.max - box.min;
}

template <typename T>
inline T area(Aabb<T, 2> const &box) noexcept {
  auto const d = extents(box);
  return d.x * d.y;
}

template <typename T>
inline T volume(Aabb<T, 3> const &box) noexcept {
  auto const d = extents(box);
  return d.x * d.y * d.z;
}

template <typename T, int N>
inline Aabb<T, N> expand(Aabb<T, N> const &box, Vec<T, N> const &amount) noexcept {
  return {box.min - amount, box.max + amount};
}

template <typename T, int N>
inline Aabb<T, N> expand(Aabb<T, N> const &box, T amount) noexcept {
  return expand(box, Vec<T, N>::all(amount));
}

template <typename T, int N>
inline Aabb<T, N> merge(Aabb<T, N> const &first, Aabb<T, N> const &second) noexcept {
  return {{min(first.min.x, second.min.x),
           min(first.min.y, second.min.y),
           min(first.min.z, second.min.z)},
          {max(first.max.x, second.max.x),
           max(first.max.y, second.max.y),
           max(first.max.z, second.max.z)}};
}

template <typename T, int N>
inline bool overlaps(Aabb<T, N> const &first, Aabb<T, N> const &second) noexcept {
  return first.min.x < second.max.x && first.min.y < second.max.y &&
         first.min.z < second.max.z && second.min.x < first.max.x &&
         second.min.y < first.max.y && second.min.z < first.max.z;
}
} // namespace marlon::math

#endif