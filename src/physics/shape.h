#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <algorithm>
#include <array>
#include <optional>
#include <type_traits>
#include <variant>

#include <math/math.h>

#include "aabb.h"
#include "contact.h"
#include "particle.h"

namespace marlon {
namespace physics {
struct Ball {
  float radius;
};

struct Capsule {
  float radius;
  float half_height;
};

struct Box {
  math::Vec3f half_extents;
};

class Shape {
public:
  Shape(Ball const &ball) noexcept : _v{ball} {}

  Shape(Capsule const &capsule) noexcept : _v{capsule} {}

  Shape(Box const &box) noexcept : _v{box} {}

  friend Aabb bounds(Shape const &shape,
                     math::Mat3x4f const &transform) noexcept;

  friend std::optional<Contact>
  particle_shape_contact(float particle_radius,
                         math::Vec3f const &particle_position,
                         Shape const &shape,
                         math::Mat3x4f const &shape_transform,
                         math::Mat3x4f const &shape_transform_inv) noexcept;

  friend std::optional<Contact>
  shape_shape_contact(Shape const &shape_a,
                      math::Mat3x4f const &transform_a,
                      math::Mat3x4f const &transform_a_inv,
                      Shape const &shape_b,
                      math::Mat3x4f const &transform_b,
                      math::Mat3x4f const &transform_b_inv) noexcept;

private:
  std::variant<Ball, Capsule, Box> _v;
};

inline Aabb bounds(Ball const &ball, math::Vec3f const &position) {
  return {.min = position - math::Vec3f::all(ball.radius),
          .max = position + math::Vec3f::all(ball.radius)};
}

inline Aabb bounds(Capsule const &capsule,
                   math::Vec3f const &position,
                   math::Vec3f const &axis) noexcept {
  auto const world_space_half_extents =
      abs(axis) + math::Vec3f::all(capsule.radius);
  return {.min = position - world_space_half_extents,
          .max = position + world_space_half_extents};
}

inline Aabb bounds(Box const &box, math::Mat3x4f const &transform) noexcept {
  auto const world_space_center = column(transform, 3);
  auto const world_space_half_extents =
      abs(transform) * math::Vec4f{box.half_extents, 0.0f};
  return Aabb{.min = world_space_center - world_space_half_extents,
              .max = world_space_center + world_space_half_extents};
}

inline Aabb bounds(Shape const &shape,
                   math::Mat3x4f const &transform) noexcept {
  return std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Ball>) {
          return bounds(arg, column(transform, 3));
        } else if constexpr (std::is_same_v<T, Capsule>) {
          return bounds(arg, column(transform, 3), column(transform, 1));
        } else {
          static_assert(std::is_same_v<T, Box>);
          return bounds(arg, transform);
        }
      },
      shape._v);
}

constexpr math::Mat3x3f solid_inertia_tensor(Ball const &ball) noexcept {
  auto const r2 = ball.radius * ball.radius;
  return 2.0f / 5.0f *
         math::Mat3x3f{{r2, 0.0f, 0.0f}, {0.0f, r2, 0.0f}, {0.0f, 0.0f, r2}};
}

constexpr math::Mat3x3f solid_inertia_tensor(Capsule const &capsule) noexcept {
  auto const r = capsule.radius;
  auto const r2 = r * r;
  auto const r3 = r2 * r;
  auto const h = capsule.half_height;
  auto const h2 = h * h;
  auto const h3 = h2 * h;
  auto const numer_parallel = r2 * (30.0f * h + 16.0f * r);
  auto const numer_perpendicular =
      20.0f * h3 + 40.0f * h2 * r + 45.0f * h * r2 + 16.0f * r3;
  auto const inverse_denom = 1.0f / (60.0f * h + 40.0f * r);
  auto const parallel = numer_parallel * inverse_denom;
  auto const perpendicular = numer_perpendicular * inverse_denom;
  return math::Mat3x3f{{perpendicular, 0.0f, 0.0f},
                       {0.0f, parallel, 0.0f},
                       {0.0f, 0.0f, perpendicular}};
}

constexpr math::Mat3x3f solid_inertia_tensor(Box const &box) noexcept {
  auto const w2 = box.half_extents[0] * box.half_extents[0];
  auto const h2 = box.half_extents[1] * box.half_extents[1];
  auto const d2 = box.half_extents[2] * box.half_extents[2];
  return 1.0f / 3.0f *
         math::Mat3x3f{{h2 + d2, 0.0f, 0.0f},
                       {0.0f, w2 + d2, 0.0f},
                       {0.0f, 0.0f, w2 + h2}};
}

constexpr math::Mat3x3f surface_inertia_tensor(Ball const &ball) noexcept {
  auto const r2 = ball.radius * ball.radius;
  return 2.0f / 3.0f *
         math::Mat3x3f{{r2, 0.0f, 0.0f}, {0.0f, r2, 0.0f}, {0.0f, 0.0f, r2}};
}

constexpr math::Mat3x3f
surface_inertia_tensor(Capsule const &capsule) noexcept {
  auto const r = capsule.radius;
  auto const r2 = r * r;
  auto const r3 = r2 * r;
  auto const h = capsule.half_height;
  auto const h2 = h * h;
  auto const h3 = h2 * h;
  auto const parallel_numer = 2 * r2 * (3 * h + 2 * r);
  auto const perpendicular_numer = 2 * h3 + 6 * h2 * r + 9 * h * r2 + 4 * r3;
  auto const inverse_denom = 1 / (6 * (h + r));
  auto const parallel = parallel_numer * inverse_denom;
  auto const perpendicular = perpendicular_numer * inverse_denom;
  return math::Mat3x3f{{perpendicular, 0.0f, 0.0f},
                       {0.0f, parallel, 0.0f},
                       {0.0f, 0.0f, perpendicular}};
}

constexpr math::Mat3x3f surface_inertia_tensor(Box const &box) noexcept {
  auto const w = box.half_extents[0];
  auto const h = box.half_extents[1];
  auto const d = box.half_extents[2];
  auto const w2 = w * w;
  auto const h2 = h * h;
  auto const d2 = d * d;
  return math::Mat3x3f{{
                           d * h * (d2 + h2) + d * w * (d2 + 3.0f * h2) +
                               h * w * (3.0f * d2 + h2),
                           0.0f,
                           0.0f,
                       },
                       {
                           0.0f,
                           w * d * (w2 + d2) + w * h * (w2 + 3.0f * d2) +
                               d * h * (3.0f * w2 + d2),
                           0.0f,
                       },
                       {
                           0.0f,
                           0.0f,
                           w * h * (w2 + h2) + w * d * (w2 + 3.0f * h2) +
                               h * d * (3.0f * w2 + h2),
                       }} /
         (3.0f * (w * h + w * d + h * d));
}

inline std::optional<Contact>
particle_shape_contact(float particle_radius,
                       math::Vec3f const &particle_position,
                       Ball const &ball,
                       math::Mat3x4f const &ball_transform,
                       math::Mat3x4f const &ball_transform_inv) noexcept {
  using namespace math;
  auto const ball_position = column(ball_transform, 3);
  auto const displacement = particle_position - ball_position;
  auto const distance2 = length_squared(displacement);
  auto const contact_distance = ball.radius + particle_radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 <= contact_distance2) {
    auto const distance = sqrt(distance2);
    auto const normal = displacement / distance;
    auto const ball_relative_position = min(ball.radius, distance) * normal;
    auto const ball_local_position =
        ball_transform_inv * Vec4f{ball_relative_position, 0.0f};
    auto const position = ball_position + ball_relative_position;
    auto const particle_local_position = particle_position - position;
    return Contact{
        .normal = normal,
        .local_positions = {particle_local_position, ball_local_position},
        .separation = distance - contact_distance,
    };
  } else {
    return std::nullopt;
  }
}

inline std::optional<Contact> particle_shape_contact(
    float /*particle_radius*/,
    math::Vec3f const & /*particle_position*/,
    Capsule const & /*capsule*/,
    math::Mat3x4f const & /*capsule_transform*/,
    math::Mat3x4f const & /*capsule_transform_inv*/) noexcept {
  return std::nullopt;
}

inline std::optional<Contact>
particle_shape_contact(float particle_radius,
                       math::Vec3f const &particle_position,
                       Box const &box,
                       math::Mat3x4f const &box_transform,
                       math::Mat3x4f const &box_transform_inverse) noexcept {
  using namespace math;
  auto const box_local_particle_position =
      box_transform_inverse * Vec4f{particle_position, 1.0f};
  if (abs(box_local_particle_position.x) - particle_radius >
          box.half_extents[0] ||
      abs(box_local_particle_position.y) - particle_radius >
          box.half_extents[1] ||
      abs(box_local_particle_position.z) - particle_radius >
          box.half_extents[2]) {
    return std::nullopt;
  }
  auto const box_local_clamped_particle_position =
      clamp(box_local_particle_position, -box.half_extents, box.half_extents);
  auto const box_local_displacement =
      box_local_particle_position - box_local_clamped_particle_position;
  auto const distance_squared = length_squared(box_local_displacement);
  auto const particle_radius_squared = particle_radius * particle_radius;
  if (distance_squared > particle_radius_squared) {
    return std::nullopt;
  } else if (distance_squared != 0.0f) {
    auto const distance = sqrt(distance_squared);
    auto const position =
        box_transform * Vec4f{box_local_clamped_particle_position, 1.0f};
    auto const particle_local_position = position - particle_position;
    auto const normal = -particle_local_position / distance;
    auto const separation = distance - particle_radius;
    return Contact{
        .normal = normal,
        .local_positions = {box_local_clamped_particle_position,
                            particle_local_position},
        .separation = separation,
    };
  } else {
    auto const face_distances = std::array<float, 6>{
        box_local_clamped_particle_position.x + box.half_extents[0],
        box.half_extents[0] - box_local_clamped_particle_position.x,
        box_local_clamped_particle_position.y + box.half_extents[1],
        box.half_extents[1] - box_local_clamped_particle_position.y,
        box_local_clamped_particle_position.z + box.half_extents[2],
        box.half_extents[2] - box_local_clamped_particle_position.z,
    };
    auto const face_index =
        std::min_element(face_distances.begin(), face_distances.end()) -
        face_distances.begin();
    auto const face_normals = std::array<Vec3f, 6>{
        -column(box_transform, 0),
        column(box_transform, 0),
        -column(box_transform, 1),
        column(box_transform, 1),
        -column(box_transform, 2),
        column(box_transform, 2),
    };
    // auto const position =
    //     box_transform * Vec4f{box_local_clamped_particle_position, 1.0f};
    // auto const particle_local_position = position - ;
    auto const normal = face_normals[face_index];
    auto const separation = -face_distances[face_index] - particle_radius;
    return Contact{
        .normal = normal,
        .local_positions = {Vec3f::zero(), box_local_clamped_particle_position},
        .separation = separation,
    };
  }
}

inline std::optional<Contact>
particle_shape_contact(float particle_radius,
                       math::Vec3f const &particle_position,
                       Shape const &shape,
                       math::Mat3x4f const &shape_transform,
                       math::Mat3x4f const &shape_transform_inv) noexcept {
  return std::visit(
      [&](auto &&arg) {
        return particle_shape_contact(particle_radius,
                                      particle_position,
                                      arg,
                                      shape_transform,
                                      shape_transform_inv);
      },
      shape._v);
}

inline std::optional<Contact>
shape_shape_contact(Ball const &b1,
                    math::Mat3x4f const &b1_transform,
                    math::Mat3x4f const &b1_transform_inv,
                    Ball const &b2,
                    math::Mat3x4f const &b2_transform,
                    math::Mat3x4f const &b2_transform_inv) {
  using namespace math;
  auto const b1_position = column(b1_transform, 3);
  auto const b2_position = column(b2_transform, 3);
  auto const displacement = b1_position - b2_position;
  auto const distance_squared = length_squared(displacement);
  auto const contact_distance = b1.radius + b2.radius;
  auto const contact_distance_squared = contact_distance * contact_distance;
  if (distance_squared > contact_distance_squared) {
    return std::nullopt;
  } else if (distance_squared != 0.0f) {
    auto const distance = sqrt(distance_squared);
    auto const normal = displacement / distance;
    auto const position =
        0.5f * (b1_position + b2_position + (b2.radius - b1.radius) * normal);
    auto const b1_relative_position = position - b1_position;
    auto const b2_relative_position = position - b2_position;
    auto const b1_local_position =
        b1_transform_inv * Vec4f{b1_relative_position, 0.0f};
    auto const b2_local_position =
        b2_transform_inv * Vec4f{b2_relative_position, 0.0f};
    return Contact{
        .normal = normal,
        .local_positions = {b1_local_position, b2_local_position},
        .separation = distance - contact_distance,
    };
  } else {
    return Contact{
        .normal = Vec3f::x_axis(),
        .local_positions = {Vec3f::zero(), Vec3f::zero()},
        .separation = -contact_distance,
    };
  }
}

inline std::optional<Contact>
shape_shape_contact(Ball const & /*ball*/,
                    math::Mat3x4f const & /*ball_transform*/,
                    math::Mat3x4f const & /*ball_transform_inv*/,
                    Capsule const & /*capsule*/,
                    math::Mat3x4f const & /*capsule_transform*/,
                    math::Mat3x4f const & /*capsule_transform_inv*/) noexcept {
  return std::nullopt;
}

inline std::optional<Contact>
shape_shape_contact(Ball const &ball,
                    math::Mat3x4f const &ball_transform,
                    math::Mat3x4f const &ball_transform_inv,
                    Box const &box,
                    math::Mat3x4f const &box_transform,
                    math::Mat3x4f const &box_transform_inv) {
  using namespace math;
  auto const ball_position = column(ball_transform, 3);
  auto const box_local_ball_position =
      box_transform_inv * Vec4f{ball_position, 1.0f};
  if (abs(box_local_ball_position.x) - ball.radius > box.half_extents[0] ||
      abs(box_local_ball_position.y) - ball.radius > box.half_extents[1] ||
      abs(box_local_ball_position.z) - ball.radius > box.half_extents[2]) {
    return std::nullopt;
  }
  auto const box_local_clamped_ball_position =
      clamp(box_local_ball_position, -box.half_extents, box.half_extents);
  auto const displacement =
      box_local_ball_position - box_local_clamped_ball_position;
  auto const distance2 = length_squared(displacement);
  if (distance2 > ball.radius * ball.radius) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const position =
        box_transform * Vec4f{box_local_clamped_ball_position, 1.0f};
    auto const ball_local_position = ball_transform_inv * Vec4f{position, 1.0f};
    auto const distance = sqrt(distance2);
    auto const normal = (ball_position - position) / distance;
    return Contact{
        .normal = normal,
        .local_positions = {ball_local_position,
                            box_local_clamped_ball_position},
        .separation = distance - ball.radius,
    };
  } else {
    auto const distances = box.half_extents - abs(box_local_ball_position);
    auto const axis = distances.x <= distances.y && distances.x <= distances.z
                          ? 0
                      : distances.y <= distances.z ? 1
                                                   : 2;
    auto const normal =
        (std::signbit(box_local_ball_position[axis]) ? -1.0f : 1.0f) *
        Vec3f{box_transform[0][axis],
              box_transform[1][axis],
              box_transform[2][axis]};
    return Contact{
        .normal = normal,
        .local_positions = {Vec3f::zero(), box_local_ball_position},
        .separation = -distances[axis] - ball.radius,
    };
  }
}

inline std::optional<Contact>
shape_shape_contact(Capsule const &capsule,
                    math::Mat3x4f const &capsule_transform,
                    math::Mat3x4f const &capsule_transform_inv,
                    Ball const &ball,
                    math::Mat3x4f const &ball_transform,
                    math::Mat3x4f const &ball_transform_inv) noexcept {
  auto result = shape_shape_contact(ball,
                                    ball_transform,
                                    ball_transform_inv,
                                    capsule,
                                    capsule_transform,
                                    capsule_transform_inv);
  if (result) {
    result->normal = -result->normal;
    std::swap(result->local_positions[0], result->local_positions[1]);
  }
  return result;
}

inline std::optional<Contact>
shape_shape_contact(Capsule const & /*c1*/,
                    math::Mat3x4f const & /*c1_transform*/,
                    math::Mat3x4f const & /*c1_transform_inv*/,
                    Capsule const & /*c2*/,
                    math::Mat3x4f const & /*c2_transform*/,
                    math::Mat3x4f const & /*c2_transform_inv*/) noexcept {
  return std::nullopt;
}

inline std::optional<Contact>
shape_shape_contact(Capsule const & /*capsule*/,
                    math::Mat3x4f const & /*capsule_transform*/,
                    math::Mat3x4f const & /*capsule_transform_inv*/,
                    Box const & /*box*/,
                    math::Mat3x4f const & /*box_transform*/,
                    math::Mat3x4f const & /*box_transform_inv*/) noexcept {
  return std::nullopt;
}

inline std::optional<Contact>
shape_shape_contact(Box const &box,
                    math::Mat3x4f const &box_transform,
                    math::Mat3x4f const &box_transform_inv,
                    Ball const &ball,
                    math::Mat3x4f const &ball_transform,
                    math::Mat3x4f const &ball_transform_inv) noexcept {
  auto result = shape_shape_contact(ball,
                                    ball_transform,
                                    ball_transform_inv,
                                    box,
                                    box_transform,
                                    box_transform_inv);
  if (result) {
    result->normal = -result->normal;
    std::swap(result->local_positions[0], result->local_positions[1]);
  }
  return result;
}

inline std::optional<Contact>
shape_shape_contact(Box const &box,
                    math::Mat3x4f const &box_transform,
                    math::Mat3x4f const &box_transform_inv,
                    Capsule const &capsule,
                    math::Mat3x4f const &capsule_transform,
                    math::Mat3x4f const &capsule_transform_inv) noexcept {
  auto result = shape_shape_contact(capsule,
                                    capsule_transform,
                                    capsule_transform_inv,
                                    box,
                                    box_transform,
                                    box_transform_inv);
  if (result) {
    result->normal = -result->normal;
    std::swap(result->local_positions[0], result->local_positions[1]);
  }
  return result;
}

inline std::optional<Contact>
shape_shape_contact(Box const &b1,
                    math::Mat3x4f const &b1_transform,
                    math::Mat3x4f const &b1_transform_inv,
                    Box const &b2,
                    math::Mat3x4f const &b2_transform,
                    math::Mat3x4f const &b2_transform_inv) {
  using namespace math;
  auto const project_to_axis =
      [](Box const &b, Mat3x4f const &m, Vec3f const &v) {
        return b.half_extents[0] * abs(dot(v, column(m, 0))) +
               b.half_extents[1] * abs(dot(v, column(m, 1))) +
               b.half_extents[2] * abs(dot(v, column(m, 2)));
      };
  auto const center_displacement =
      column(b1_transform, 3) - column(b2_transform, 3);
  auto const separation_on_axis = [&](Vec3f const &v) {
    auto const b1_projection = project_to_axis(b1, b1_transform, v);
    auto const b2_projection = project_to_axis(b2, b2_transform, v);
    auto const distance = abs(dot(center_displacement, v));
    return distance - (b1_projection + b2_projection);
  };
  auto const edge_edge_contact_position = [&](Vec3f const &p1,
                                              Vec3f const &d1,
                                              float s1,
                                              Vec3f const &p2,
                                              Vec3f const &d2,
                                              float s2,
                                              bool use_one) -> Vec3f {
    auto const d1_length2 = length_squared(d1);
    auto const d2_length2 = length_squared(d2);
    auto const d1_dot_d2 = dot(d1, d2);
    auto const p_diff = p1 - p2;
    auto const dp_sta_1 = dot(d1, p_diff);
    auto const dp_sta_2 = dot(d2, p_diff);
    auto const denominator = d1_length2 * d2_length2 - d1_dot_d2 * d1_dot_d2;
    if (abs(denominator) < 0.0001f) {
      return use_one ? p1 : p2;
    }
    auto const inverse_denominator = 1.0f / denominator;
    auto const mua =
        (d1_dot_d2 * dp_sta_2 - d2_length2 * dp_sta_1) * inverse_denominator;
    auto const mub =
        (d1_length2 * dp_sta_2 - d1_dot_d2 * dp_sta_1) * inverse_denominator;
    if (abs(mua) > s1 || abs(mub) > s2) {
      return use_one ? p1 : p2;
    } else {
      auto const c1 = d1 * mua + p1;
      auto const c2 = d2 * mub + p2;
      return 0.5f * (c1 + c2);
    }
  };
  auto separating_axes = std::array<Vec3f, 15>{
      column(b1_transform, 0),
      column(b1_transform, 1),
      column(b1_transform, 2),
      column(b2_transform, 0),
      column(b2_transform, 1),
      column(b2_transform, 2),
      cross(column(b1_transform, 0), column(b2_transform, 0)),
      cross(column(b1_transform, 0), column(b2_transform, 1)),
      cross(column(b1_transform, 0), column(b2_transform, 2)),
      cross(column(b1_transform, 1), column(b2_transform, 0)),
      cross(column(b1_transform, 1), column(b2_transform, 1)),
      cross(column(b1_transform, 1), column(b2_transform, 2)),
      cross(column(b1_transform, 2), column(b2_transform, 0)),
      cross(column(b1_transform, 2), column(b2_transform, 1)),
      cross(column(b1_transform, 2), column(b2_transform, 2)),
  };
  auto best_separation = -std::numeric_limits<float>::max();
  auto best_separating_axis_index = -1;
  for (auto separating_axis_index = 0; separating_axis_index != 6;
       ++separating_axis_index) {
    auto const &separating_axis = separating_axes[separating_axis_index];
    auto const separation = separation_on_axis(separating_axis);
    if (separation > 0.0f) {
      return std::nullopt;
    }
    if (separation > best_separation) {
      best_separation = separation;
      best_separating_axis_index = separating_axis_index;
    }
  }
  auto const best_basis_separating_axis_index = best_separating_axis_index;
  for (auto separating_axis_index = 6; separating_axis_index != 15;
       ++separating_axis_index) {
    auto &separating_axis = separating_axes[separating_axis_index];
    auto const separating_axis_length2 = length_squared(separating_axis);
    if (separating_axis_length2 < 0.001f) {
      continue;
    }
    auto const separating_axis_length = std::sqrt(separating_axis_length2);
    separating_axis /= separating_axis_length;
    auto const separation = separation_on_axis(separating_axis);
    if (separation > 0.0f) {
      return std::nullopt;
    }
    if (separation > best_separation) {
      best_separation = separation;
      best_separating_axis_index = separating_axis_index;
    }
  }
  if (best_separating_axis_index < 3) {
    // vertex of box two on face of box one
    auto const axis = separating_axes[best_separating_axis_index];
    auto const normal = dot(axis, center_displacement) >= 0.0f ? axis : -axis;
    auto const b2_local_position =
        Vec3f{dot(separating_axes[3], normal) >= 0.0f ? b2.half_extents[0]
                                                      : -b2.half_extents[0],
              dot(separating_axes[4], normal) >= 0.0f ? b2.half_extents[1]
                                                      : -b2.half_extents[1],
              dot(separating_axes[5], normal) >= 0.0f ? b2.half_extents[2]
                                                      : -b2.half_extents[2]};
    auto const position = b2_transform * Vec4f{b2_local_position, 1.0f};
    auto const b1_local_position = b1_transform_inv * Vec4f{position, 1.0f};
    return Contact{
        .normal = normal,
        .local_positions = {b1_local_position, b2_local_position},
        .separation = best_separation,
    };
  } else if (best_separating_axis_index < 6) {
    // vertex of box one on face of box two
    auto const axis = separating_axes[best_separating_axis_index];
    auto const normal = dot(axis, center_displacement) >= 0.0f ? axis : -axis;
    auto const b1_local_position =
        Vec3f{dot(separating_axes[0], normal) >= 0.0f ? -b1.half_extents[0]
                                                      : b1.half_extents[0],
              dot(separating_axes[1], normal) >= 0.0f ? -b1.half_extents[1]
                                                      : b1.half_extents[1],
              dot(separating_axes[2], normal) >= 0.0f ? -b1.half_extents[2]
                                                      : b1.half_extents[2]};
    auto const position = b1_transform * Vec4f{b1_local_position, 1.0f};
    auto const b2_local_position = b2_transform_inv * Vec4f{position, 1.0f};
    return Contact{
        .normal = normal,
        .local_positions = {b1_local_position, b2_local_position},
        .separation = best_separation,
    };
  } else {
    auto const b1_axis_index = (best_separating_axis_index - 6) / 3;
    auto const b2_axis_index = (best_separating_axis_index - 6) % 3;
    auto const axis = separating_axes[best_separating_axis_index];
    auto const normal = dot(axis, center_displacement) < 0.0f ? -axis : axis;
    auto const b1_extents =
        Vec3f{b1.half_extents[0], b1.half_extents[1], b1.half_extents[2]};
    auto const on_b1_edge =
        b1_transform *
        Vec4f{Vec3f{[&](int axis_index) {
                if (axis_index == b1_axis_index) {
                  return 0.0f;
                } else {
                  return dot(separating_axes[axis_index], normal) > 0.0f
                             ? -b1_extents[axis_index]
                             : b1_extents[axis_index];
                }
              }},
              1.0f};
    auto const b2_extents =
        Vec3f{b2.half_extents[0], b2.half_extents[1], b2.half_extents[2]};
    auto const on_b2_edge =
        b2_transform *
        Vec4f{Vec3f{[&](int axis_index) {
                if (axis_index == b2_axis_index) {
                  return 0.0f;
                } else {
                  return dot(separating_axes[axis_index + 3], normal) < 0.0f
                             ? -b2_extents[axis_index]
                             : b2_extents[axis_index];
                }
              }},
              1.0f};
    auto const position =
        edge_edge_contact_position(on_b1_edge,
                                   separating_axes[b2_axis_index],
                                   b1_extents[b1_axis_index],
                                   on_b2_edge,
                                   separating_axes[b2_axis_index + 3],
                                   b2_extents[b2_axis_index],
                                   best_basis_separating_axis_index >= 3);
    auto const b1_local_position = b1_transform_inv * Vec4f{position, 1.0f};
    auto const b2_local_position = b2_transform_inv * Vec4f{position, 1.0f};
    return Contact{
        .normal = normal,
        .local_positions = {b1_local_position, b2_local_position},
        .separation = best_separation,
    };
  }
}

inline std::optional<Contact>
shape_shape_contact(Shape const &shape_a,
                    math::Mat3x4f const &transform_a,
                    math::Mat3x4f const &transform_a_inv,
                    Shape const &shape_b,
                    math::Mat3x4f const &transform_b,
                    math::Mat3x4f const &transform_b_inv) noexcept {
  return std::visit(
      [&](auto &&a) {
        return std::visit(
            [&](auto &&b) {
              return shape_shape_contact(a,
                                         transform_a,
                                         transform_a_inv,
                                         b,
                                         transform_b,
                                         transform_b_inv);
            },
            shape_b._v);
      },
      shape_a._v);
}
} // namespace physics
} // namespace marlon

#endif