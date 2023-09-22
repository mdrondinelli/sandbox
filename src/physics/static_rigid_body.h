#ifndef MARLON_PHYSICS_STATIC_RIGID_BODY_H
#define MARLON_PHYSICS_STATIC_RIGID_BODY_H

#include <cstdint>

#include "../math/quat.h"
#include "../math/vec.h"
#include "shape.h"

namespace marlon {
namespace physics {
struct Static_rigid_body_handle {
  std::size_t value;
};

struct Static_rigid_body_create_info {
  std::uint64_t collision_flags{};
  std::uint64_t collision_mask{};
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  Shape shape;
  Material material;
};

constexpr bool operator==(Static_rigid_body_handle lhs,
                          Static_rigid_body_handle rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Static_rigid_body_handle> {
  std::size_t operator()(
      marlon::physics::Static_rigid_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value);
  }
};
} // namespace std

#endif