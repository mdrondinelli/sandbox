#ifndef MARLON_PHYSICS_STATIC_BODY_H
#define MARLON_PHYSICS_STATIC_BODY_H

#include <cstdint>

#include "../math/quat.h"
#include "../math/vec.h"
#include "handle.h"
#include "material.h"
#include "shape.h"

namespace marlon {
namespace physics {
struct Static_body_handle {
  Object_handle value;
};

struct Static_body_create_info {
  Shape shape;
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
};

constexpr bool operator==(Static_body_handle lhs,
                          Static_body_handle rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Static_body_handle> {
  std::size_t
  operator()(marlon::physics::Static_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value);
  }
};
} // namespace std

#endif