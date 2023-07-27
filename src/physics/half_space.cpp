#include "half_space.h"

namespace marlon {
namespace physics {
Half_space::Half_space(math::Vec3f const &normal) noexcept : _normal{normal} {}

std::optional<Contact>
Half_space::collide_particle(math::Vec3f const &shape_position,
                             math::Quatf const & /*shape_orientation*/,
                             math::Vec3f const &particle_position,
                             float particle_radius) const noexcept {
  auto const displacement = particle_position - shape_position;
  auto const distance = math::dot(_normal, displacement);
  if (distance <= particle_radius) {
    return Contact{.position = -distance * _normal + particle_position,
                   .normal = _normal,
                   .depth = particle_radius - distance};
  } else {
    return std::nullopt;
  }
}
} // namespace physics
} // namespace marlon