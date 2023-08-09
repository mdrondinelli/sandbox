#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <optional>

#include "../math/mat.h"
#include "../math/quat.h"
#include "../math/vec.h"
#include "bounding_box.h"
#include "contact.h"

namespace marlon {
namespace physics {
class Shape {
public:
  virtual ~Shape() = default;

  virtual Bounding_box
  get_bounds(math::Mat3x4f const &transform) const noexcept = 0;

  virtual std::optional<Particle_contact>
  collide_particle(math::Mat3x4f const &shape_transform,
                   math::Mat3x4f const &shape_transform_inverse,
                   math::Vec3f const &particle_position,
                   float particle_radius) const noexcept = 0;
};
} // namespace physics
} // namespace marlon

#endif