#ifndef MARLON_PHYSICS_BALL_H
#define MARLON_PHYSICS_BALL_H

#include <optional>

#include "bounding_box.h"
#include "particle.h"

namespace marlon {
namespace physics {
struct Ball {
  float radius;
};

inline Bounding_box bounds(Ball const &ball, math::Vec3f const &position) {
  return {.min = position - math::Vec3f::all(ball.radius),
          .max = position + math::Vec3f::all(ball.radius)};
}

inline std::optional<Particle_contact>
find_particle_contact(math::Vec3f const &particle_position,
                      float particle_radius, Ball const &ball,
                      math::Vec3f const &ball_position) noexcept {
  auto const displacement = particle_position - ball_position;
  auto const distance2 = math::length2(displacement);
  auto const contact_distance = ball.radius + particle_radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 <= contact_distance2) {
    auto const distance = std::sqrt(distance2);
    auto const normal = displacement / distance;
    return Particle_contact{.normal = normal,
                            .separation = distance - contact_distance};
  } else {
    return std::nullopt;
  }
}
} // namespace physics
} // namespace marlon

#endif