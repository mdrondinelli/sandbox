#ifndef MARLON_MATH_AABB_BOUNDS_H
#define MARLON_MATH_AABB_BOUNDS_H

#include "vec.h"

namespace marlon::math {
struct Aabb {
  Vec3f min;
  Vec3f max;
};

inline Vec3f center(Aabb const &box) noexcept {
  return 0.5f * (box.min + box.max);
}

inline Vec3f extents(Aabb const &box) noexcept { return box.max - box.min; }

inline float volume(Aabb const &box) noexcept {
  auto const d = extents(box);
  return d.x * d.y * d.z;
}

inline Aabb expand(Aabb const &box, Vec3f const &amount) noexcept {
  return {box.min - amount, box.max + amount};
}

inline Aabb expand(Aabb const &box, float amount) noexcept {
  return expand(box, Vec3f::all(amount));
}

inline Aabb merge(Aabb const &first, Aabb const &second) noexcept {
  return {{min(first.min.x, second.min.x),
           min(first.min.y, second.min.y),
           min(first.min.z, second.min.z)},
          {max(first.max.x, second.max.x),
           max(first.max.y, second.max.y),
           max(first.max.z, second.max.z)}};
}

inline bool overlaps(Aabb const &first, Aabb const &second) noexcept {
  return first.min.x < second.max.x && first.min.y < second.max.y &&
         first.min.z < second.max.z && second.min.x < first.max.x &&
         second.min.y < first.max.y && second.min.z < first.max.z;
}
} // namespace marlon::math

#endif