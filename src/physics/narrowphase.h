#ifndef MARLON_PHYSICS_NARROWPHASE_H
#define MARLON_PHYSICS_NARROWPHASE_H

#include <optional>

#include "../math/math.h"
#include "contact.h"
#include "particle.h"
#include "shape.h"

namespace marlon {
namespace physics {
std::optional<Contact>
object_object_contact(Particle_data const &first,
                      Particle_data const &second) noexcept {
  using namespace math;
  auto const displacement = first.position() - second.position();
  auto const distance_squared = length_squared(displacement);
  auto const contact_distance = first.radius() + second.radius();
  auto const contact_distance_squared = contact_distance * contact_distance;
  if (distance_squared < contact_distance_squared) {
    auto const [normal, separation] = [&]() {
      if (distance_squared == 0.0f) {
        // particles coincide, pick arbitrary contact normal
        auto const contact_normal = math::Vec3f{1.0f, 0.0f, 0.0f};
        auto const separation = -contact_distance;
        return std::tuple{contact_normal, separation};
      } else {
        auto const distance = sqrt(distance_squared);
        auto const normal = displacement / distance;
        auto const separation = distance - contact_distance;
        return std::tuple{normal, separation};
      }
    }();
    auto const position = 0.5f * (first.position() + second.position() +
                                  (second.radius() - first.radius()) * normal);
    return Contact{
        .normal = normal,
        .local_positions = {position - first.position(),
                            position - second.position()},
        .separation = separation,
    };
  } else {
    return std::nullopt;
  }
}

std::optional<Contact>
object_object_contact(Particle_data const &first,
                      Rigid_body_data const &second) noexcept {
  using namespace math;
  auto const transform =
      Mat3x4f::rigid(second.position(), second.orientation());
  auto const transform_inv = rigid_inverse(transform);
  return particle_shape_contact(first.radius(),
                                first.position(),
                                second.shape(),
                                transform,
                                transform_inv);
}

std::optional<Contact>
object_object_contact(Particle_data const &first,
                      Static_body_data const &second) noexcept {
  using namespace math;
  auto const transform =
      Mat3x4f::rigid(second.position(), second.orientation());
  auto const inverse_transform = rigid_inverse(transform);
  return particle_shape_contact(first.radius(),
                                first.position(),
                                second.shape(),
                                transform,
                                inverse_transform);
}

std::optional<Contact>
object_object_contact(Rigid_body_data const &first,
                      Rigid_body_data const &second) noexcept {
  using namespace math;
  auto const transforms =
      std::pair{Mat3x4f::rigid(first.position(), first.orientation()),
                Mat3x4f::rigid(second.position(), second.orientation())};
  auto const inverse_transforms = std::pair{rigid_inverse(transforms.first),
                                            rigid_inverse(transforms.second)};
  return shape_shape_contact(first.shape(),
                             transforms.first,
                             inverse_transforms.first,
                             second.shape(),
                             transforms.second,
                             inverse_transforms.second);
}

std::optional<Contact>
object_object_contact(Rigid_body_data const &first,
                      Static_body_data const &second) noexcept {
  using namespace math;
  auto const transforms = std::array<Mat3x4f, 2>{
      Mat3x4f::rigid(first.position(), first.orientation()),
      Mat3x4f::rigid(second.position(), second.orientation()),
  };
  auto const inverse_transforms = std::array<Mat3x4f, 2>{
      rigid_inverse(transforms[0]),
      rigid_inverse(transforms[1]),
  };
  return shape_shape_contact(first.shape(),
                             transforms[0],
                             inverse_transforms[0],
                             second.shape(),
                             transforms[1],
                             inverse_transforms[1]);
  return std::nullopt;
}
} // namespace physics
} // namespace marlon

#endif