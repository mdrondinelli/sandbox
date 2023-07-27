#include "half_space.h"

namespace marlon {
namespace physics {
Half_space::Half_space(math::Vec3f const &normal) noexcept : _normal{normal} {}

std::optional<Contact>
Half_space::collide_particle(math::Mat3x4f const &shape_transform,
                             math::Vec3f const &particle_position,
                             float particle_radius) const noexcept {
  auto const plane_position = math::Vec3f{
      shape_transform[0][3], shape_transform[1][3], shape_transform[2][3]};
  auto const plane_normal = math::Vec3f{
      shape_transform[0][0] * _normal[0] + shape_transform[0][1] * _normal[1] +
          shape_transform[0][2] * _normal[2],
      shape_transform[1][0] * _normal[0] + shape_transform[1][1] * _normal[1] +
          shape_transform[1][2] * _normal[2],
      shape_transform[2][0] * _normal[0] + shape_transform[2][1] * _normal[1] +
          shape_transform[2][2] * _normal[2]};
  auto const displacement = particle_position - plane_position;
  auto const distance = math::dot(plane_normal, displacement);
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