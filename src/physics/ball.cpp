#include "ball.h"

namespace marlon {
namespace physics {
Ball::Ball(float radius) noexcept : _radius{radius} {}

std::optional<Contact>
Ball::collide_particle(math::Vec3f const &shape_position,
                       math::Quatf const & /*shape_orientation*/,
                       math::Vec3f const &particle_position,
                       float particle_radius) const noexcept {
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