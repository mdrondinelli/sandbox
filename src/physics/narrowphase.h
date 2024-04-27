#ifndef MARLON_PHYSICS_NARROWPHASE_H
#define MARLON_PHYSICS_NARROWPHASE_H

#include <optional>

#include "../math/math.h"
#include "contact.h"
#include "particle.h"
#include "shape.h"

namespace marlon {
namespace physics {
std::optional<Contact> object_object_contact(
    std::pair<Particle_data *, Particle_data *> data) noexcept {
  using namespace math;
  auto const displacement = data.first->position - data.second->position;
  auto const distance_squared = length_squared(displacement);
  auto const contact_distance = data.first->radius + data.second->radius;
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
    auto const position =
        0.5f * (data.first->position + data.second->position +
                (data.second->radius - data.first->radius) * normal);
    return Contact{
        .normal = normal,
        .local_positions = {position - data.first->position,
                            position - data.second->position},
        .separation_bias = separation,
    };
  } else {
    return std::nullopt;
  }
}

std::optional<Contact> object_object_contact(
    std::pair<Particle_data *, Rigid_body_data *> data) noexcept {
  using namespace math;
  auto const transform =
      Mat3x4f::rigid(data.second->position, data.second->orientation);
  auto const transform_inv = rigid_inverse(transform);
  return particle_shape_contact(data.first->radius,
                                data.first->position,
                                data.second->shape,
                                transform,
                                transform_inv);
}

std::optional<Contact> object_object_contact(
    std::pair<Particle_data *, Static_body_data *> data) noexcept {
  using namespace math;
  auto const transform =
      Mat3x4f::rigid(data.second->position, data.second->orientation);
  auto const inverse_transform = rigid_inverse(transform);
  return particle_shape_contact(data.first->radius,
                                data.first->position,
                                data.second->shape,
                                transform,
                                inverse_transform);
}

std::optional<Contact> object_object_contact(
    std::pair<Rigid_body_data *, Rigid_body_data *> data) noexcept {
  using namespace math;
  auto const transforms = std::pair{
      Mat3x4f::rigid(data.first->position, data.first->orientation),
      Mat3x4f::rigid(data.second->position, data.second->orientation)};
  auto const inverser_transforms = std::pair{rigid_inverse(transforms.first),
                                             rigid_inverse(transforms.second)};
  return shape_shape_contact(data.first->shape,
                             transforms.first,
                             inverser_transforms.first,
                             data.second->shape,
                             transforms.second,
                             inverser_transforms.second);
}

std::optional<Contact> object_object_contact(
    std::pair<Rigid_body_data *, Static_body_data *> data) noexcept {
  using namespace math;
  auto const transforms = std::array<Mat3x4f, 2>{
      Mat3x4f::rigid(data.first->position, data.first->orientation),
      Mat3x4f::rigid(data.second->position, data.second->orientation),
  };
  auto const inverse_transforms = std::array<Mat3x4f, 2>{
      rigid_inverse(transforms[0]),
      rigid_inverse(transforms[1]),
  };
  return shape_shape_contact(data.first->shape,
                             transforms[0],
                             inverse_transforms[0],
                             data.second->shape,
                             transforms[1],
                             inverse_transforms[1]);
  return std::nullopt;
}
} // namespace physics
} // namespace marlon

#endif