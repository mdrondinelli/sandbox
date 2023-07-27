#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <optional>

#include "../math/quat.h"
#include "../math/vec.h"
#include "contact.h"

namespace marlon {
namespace physics {
class Shape {
public:
  virtual ~Shape() = default;

  virtual std::optional<Contact>
  collide_particle(math::Vec3f const &shape_position,
                   math::Quatf const &shape_orientation,
                   math::Vec3f const &particle_position,
                   float particle_radius) const noexcept = 0;
};
} // namespace physics
} // namespace marlon

#endif