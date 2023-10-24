#ifndef MARLON_PHYSICS_DYNAMIC_RIGID_BODY_H
#define MARLON_PHYSICS_DYNAMIC_RIGID_BODY_H

#include <cstdint>

#include "../math/quat.h"
#include "../math/vec.h"
#include "handle.h"
#include "material.h"
#include "shape.h"

namespace marlon {
namespace physics {
class Dynamic_rigid_body_motion_callback;

struct Dynamic_rigid_body_handle {
  Object_handle_t value;
};

struct Dynamic_rigid_body_create_info {
  Dynamic_rigid_body_motion_callback *motion_callback;
  std::uint64_t collision_flags{};
  std::uint64_t collision_mask{};
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec3f angular_velocity{math::Vec3f::zero()};
  float mass{1.0f};
  math::Mat3x3f inertia_tensor{math::Mat3x3f::identity()};
  Shape shape;
  Material material;
};

struct Dynamic_rigid_body_motion_event {
  Dynamic_rigid_body_handle handle;
  math::Vec3f position;
  math::Quatf orientation;
};

class Dynamic_rigid_body_motion_callback {
public:
  virtual ~Dynamic_rigid_body_motion_callback() = default;

  virtual void on_dynamic_rigid_body_motion(
      Dynamic_rigid_body_motion_event const &event) = 0;
};

constexpr bool operator==(Dynamic_rigid_body_handle lhs,
                          Dynamic_rigid_body_handle rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Dynamic_rigid_body_handle> {
  std::size_t operator()(
      marlon::physics::Dynamic_rigid_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value);
  }
};
} // namespace std

#endif