#ifndef MARLON_PHYSICS_BOX_H
#define MARLON_PHYSICS_BOX_H

#include "shape.h"

namespace marlon {
namespace physics {
class Box : public Shape {
public:
  explicit Box(float half_width, float half_height, float half_depth) noexcept;

  Bounding_box get_bounds(math::Mat3x4f const &transform) const noexcept final;

  std::optional<Contact>
  collide_particle(math::Mat3x4f const &shape_transform,
                   math::Mat3x4f const &shape_transform_inverse,
                   math::Vec3f const &particle_position,
                   float particle_radius) const noexcept final;

private:
  float _half_width;
  float _half_height;
  float _half_depth;
};
} // namespace physics
} // namespace marlon

#endif