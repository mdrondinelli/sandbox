#include "box.h"

#include <algorithm>
#include <array>

namespace marlon {
namespace physics {
Box::Box(float half_width, float half_height, float half_depth) noexcept
    : _half_width{half_width}, _half_height{half_height},
      _half_depth{half_depth} {}

Bounding_box Box::get_bounds(math::Mat3x4f const &transform) const noexcept {
  auto const shape_space_points = std::array<math::Vec3f, 8>{
      math::Vec3f{-_half_width, -_half_height, -_half_depth},
      math::Vec3f{-_half_width, -_half_height, _half_depth},
      math::Vec3f{-_half_width, _half_height, -_half_depth},
      math::Vec3f{-_half_width, _half_height, _half_depth},
      math::Vec3f{_half_width, -_half_height, -_half_depth},
      math::Vec3f{_half_width, -_half_height, _half_depth},
      math::Vec3f{_half_width, _half_height, -_half_depth},
      math::Vec3f{_half_width, _half_height, _half_depth}};
  auto world_space_points = std::array<math::Vec3f, 8>{};
  for (auto i = 0; i < 8; ++i) {
    auto const &shape_space_point = shape_space_points[i];
    auto &world_space_point = world_space_points[i];
    world_space_point.x = transform[0][0] * shape_space_point.x +
                          transform[0][1] * shape_space_point.y +
                          transform[0][2] * shape_space_point.z +
                          transform[0][3];
    world_space_point.y = transform[1][0] * shape_space_point.x +
                          transform[1][1] * shape_space_point.y +
                          transform[1][2] * shape_space_point.z +
                          transform[1][3];
    world_space_point.z = transform[2][0] * shape_space_point.x +
                          transform[2][1] * shape_space_point.y +
                          transform[2][2] * shape_space_point.z +
                          transform[2][3];
  }
  auto bounds = Bounding_box{world_space_points[0], world_space_points[0]};
  for (auto i = 1; i < 8; ++i) {
    auto const &world_space_point = world_space_points[i];
    bounds.min.x = std::min(bounds.min.x, world_space_point.x);
    bounds.min.y = std::min(bounds.min.y, world_space_point.y);
    bounds.min.z = std::min(bounds.min.z, world_space_point.z);
    bounds.max.x = std::max(bounds.max.x, world_space_point.x);
    bounds.max.y = std::max(bounds.max.y, world_space_point.y);
    bounds.max.z = std::max(bounds.max.z, world_space_point.z);
  }
  return bounds;
}

std::optional<Contact>
Box::collide_particle(math::Mat3x4f const &shape_transform,
                      math::Mat3x4f const &shape_transform_inverse,
                      math::Vec3f const &particle_position,
                      float particle_radius) const noexcept {
  auto const shape_space_particle_position =
      math::Vec3f{shape_transform_inverse[0][0] * particle_position[0] +
                      shape_transform_inverse[0][1] * particle_position[1] +
                      shape_transform_inverse[0][2] * particle_position[2] +
                      shape_transform_inverse[0][3],
                  shape_transform_inverse[1][0] * particle_position[0] +
                      shape_transform_inverse[1][1] * particle_position[1] +
                      shape_transform_inverse[1][2] * particle_position[2] +
                      shape_transform_inverse[1][3],
                  shape_transform_inverse[2][0] * particle_position[0] +
                      shape_transform_inverse[2][1] * particle_position[1] +
                      shape_transform_inverse[2][2] * particle_position[2] +
                      shape_transform_inverse[2][3]};
  auto const shape_space_clamped_particle_position = math::Vec3f{
      std::clamp(shape_space_particle_position.x, -_half_width, _half_width),
      std::clamp(shape_space_particle_position.y, -_half_height, _half_height),
      std::clamp(shape_space_particle_position.z, -_half_depth, _half_depth)};
  auto const displacement =
      shape_space_particle_position - shape_space_clamped_particle_position;
  auto const distance2 = math::length2(displacement);
  auto const particle_radius2 = particle_radius * particle_radius;
  if (distance2 <= particle_radius2) {
    auto const face_distances = std::array<float, 6>{
        shape_space_clamped_particle_position.x + _half_width,
        _half_width - shape_space_clamped_particle_position.x,
        shape_space_clamped_particle_position.y + _half_height,
        _half_height - shape_space_clamped_particle_position.y,
        shape_space_clamped_particle_position.z + _half_depth,
        _half_depth - shape_space_clamped_particle_position.z};
    auto const face_normals = std::array<math::Vec3f, 6>{
        -math::Vec3f{shape_transform[0][0], shape_transform[1][0],
                     shape_transform[2][0]},
        math::Vec3f{shape_transform[0][0], shape_transform[1][0],
                    shape_transform[2][0]},
        -math::Vec3f{shape_transform[0][1], shape_transform[1][1],
                     shape_transform[2][1]},
        math::Vec3f{shape_transform[0][1], shape_transform[1][1],
                    shape_transform[2][1]},
        -math::Vec3f{shape_transform[0][2], shape_transform[1][2],
                     shape_transform[2][2]},
        math::Vec3f{shape_transform[0][2], shape_transform[1][2],
                    shape_transform[2][2]}};
    auto const face_index =
        std::min_element(face_distances.begin(), face_distances.end()) -
        face_distances.begin();
    auto const depth = face_distances[face_index] + particle_radius;
    auto const normal = face_normals[face_index];
    auto const position = math::Vec3f{
        shape_transform[0][0] * shape_space_clamped_particle_position[0] +
            shape_transform[0][1] * shape_space_clamped_particle_position[1] +
            shape_transform[0][2] * shape_space_clamped_particle_position[2] +
            shape_transform[0][3],
        shape_transform[1][0] * shape_space_clamped_particle_position[0] +
            shape_transform[1][1] * shape_space_clamped_particle_position[1] +
            shape_transform[1][2] * shape_space_clamped_particle_position[2] +
            shape_transform[1][3],
        shape_transform[2][0] * shape_space_clamped_particle_position[0] +
            shape_transform[2][1] * shape_space_clamped_particle_position[1] +
            shape_transform[2][2] * shape_space_clamped_particle_position[2] +
            shape_transform[2][3]};
    return Contact{.position = position, .normal = normal, .depth = depth};
  } else {
    return std::nullopt;
  }
}
} // namespace physics
} // namespace marlon