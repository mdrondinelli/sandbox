#include "ball.h"

namespace marlon {
namespace physics {
Ball::Ball(float radius) noexcept : _radius{radius} {}

Bounding_box Ball::get_bounds(math::Mat3x4f const &transform) const noexcept {
  return {{transform[0][3] - _radius, transform[1][3] - _radius,
           transform[2][3] - _radius},
          {transform[0][3] + _radius, transform[1][3] + _radius,
           transform[2][3] + _radius}};
}

std::optional<Contact>
Ball::collide_particle(math::Mat3x4f const &shape_transform,
                       math::Mat3x4f const & /*shape_transform_inverse*/,
                       math::Vec3f const &particle_position,
                       float particle_radius) const noexcept {
  auto const shape_position = math::Vec3f{
      shape_transform[0][3], shape_transform[1][3], shape_transform[2][3]};
  auto const displacement = particle_position - shape_position;
  auto const distance2 = math::length2(displacement);
  auto const contact_distance = _radius + particle_radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 <= contact_distance2) {
    auto const distance = std::sqrt(distance2);
    auto const normal = displacement / distance;
    return Contact{.position = -particle_radius * normal + particle_position,
                   .normal = normal,
                   .depth = contact_distance - distance};
  } else {
    return std::nullopt;
  }
}
} // namespace physics
} // namespace marlon