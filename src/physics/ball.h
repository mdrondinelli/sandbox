#ifndef MARLON_PHYSICS_BALL_H
#define MARLON_PHYSICS_BALL_H

#include "shape.h"

namespace marlon {
namespace physics {
class Ball : public Shape {
public:
  explicit Ball(float radius) noexcept;

  std::optional<Contact>
  collide_particle(math::Mat3x4f const &shape_transform,
                   math::Mat3x4f const &shape_transform_inverse,
                   math::Vec3f const &particle_position,
                   float particle_radius) const noexcept final;

private:
  float _radius;
};
} // namespace physics
} // namespace marlon

#endif