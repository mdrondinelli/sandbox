#ifndef MARLON_PHYSICS_BOUNDING_BOX_H
#define MARLON_PHYSICS_BOUNDING_BOX_H

#include "../math/vec.h"

namespace marlon {
namespace physics {
struct Bounding_box {
  math::Vec3f min;
  math::Vec3f max;
};

inline float volume(Bounding_box const &box) noexcept {
  auto const d = box.max - box.min;
  return d.x * d.y * d.z;
}

inline math::Vec3f centroid(Bounding_box const &box) noexcept {
  return 0.5f * (box.min + box.max);
}

inline Bounding_box inflate(Bounding_box const &box, float amount) noexcept {
  return {box.min - math::Vec3f{amount, amount, amount},
          box.max + math::Vec3f{amount, amount, amount}};
}

inline Bounding_box merge(Bounding_box const &first,
                          Bounding_box const &second) noexcept {
  return {
      {std::min(first.min.x, second.min.x), std::min(first.min.y, second.min.y),
       std::min(first.min.z, second.min.z)},
      {std::max(first.max.x, second.max.x), std::max(first.max.y, second.max.y),
       std::max(first.max.z, second.max.z)}};
}

inline bool overlaps(Bounding_box const &first,
                     Bounding_box const &second) noexcept {
  return first.min.x < second.max.x && first.min.y < second.max.y &&
         first.min.z < second.max.z && second.min.x < first.max.x &&
         second.min.y < first.max.y && second.min.z < first.max.z;
}
} // namespace physics
} // namespace marlon

#endif