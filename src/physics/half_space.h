#ifndef MARLON_PHYSICS_HALF_SPACE_H
#define MARLON_PHYSICS_HALF_SPACE_H

#include "shape.h"

namespace marlon {
namespace physics {
class Half_space : public Shape {
public:
  explicit Half_space(math::Vec3f const &normal) noexcept;

  std::optional<Contact>
  collide_particle(math::Mat3x4f const &shape_transform,
                   math::Vec3f const &particle_position,
                   float particle_radius) const noexcept final;

private:
  math::Vec3f _normal;
};
} // namespace physics
} // namespace marlon

#endif