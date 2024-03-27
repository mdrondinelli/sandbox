#ifndef MARLON_PHYSICS_RIGID_BODY_H
#define MARLON_PHYSICS_RIGID_BODY_H

#include <cstdint>

#include "../math/quat.h"
#include "../math/vec.h"
#include "handle.h"
#include "material.h"
#include "shape.h"

namespace marlon {
namespace physics {
class Rigid_body_motion_callback;

struct Rigid_body_handle {
  Object_handle value;
};

struct Rigid_body_create_info {
  Rigid_body_motion_callback *motion_callback{};
  Shape shape;
  float mass{1.0f};
  math::Mat3x3f inertia_tensor{math::Mat3x3f::identity()};
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec3f angular_velocity{math::Vec3f::zero()};
};

struct Rigid_body_motion_event {
  Rigid_body_handle handle;
  math::Vec3f position;
  math::Quatf orientation;
};

class Rigid_body_motion_callback {
public:
  virtual ~Rigid_body_motion_callback() = default;

  virtual void on_rigid_body_motion(Rigid_body_motion_event const &event) = 0;
};

constexpr bool operator==(Rigid_body_handle lhs,
                          Rigid_body_handle rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Rigid_body_handle> {
  std::size_t
  operator()(marlon::physics::Rigid_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value);
  }
};
} // namespace std

#endif