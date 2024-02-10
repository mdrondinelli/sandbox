#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <algorithm>
#include <array>
#include <optional>
#include <type_traits>
#include <variant>

#include "../math/mat.h"
#include "../math/vec.h"
#include "aabb.h"
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

struct Positionless_contact_geometry {
  math::Vec3f normal;
  float separation;
};

struct Positionful_contact_geometry {
  math::Vec3f position;
  math::Vec3f normal;
  float separation;
};

template <typename T> struct Support_function;

class Shape {
public:
  Shape(Ball const &ball) noexcept : _v{ball} {}

  Shape(Capsule const &capsule) noexcept : _v{capsule} {}

  Shape(Box const &box) noexcept : _v{box} {}

  friend Aabb bounds(Shape const &shape,
                     math::Mat3x4f const &transform) noexcept;

  friend std::optional<Positionless_contact_geometry>
  particle_shape_positionless_contact_geometry(
      math::Vec3f const &particle_position, float particle_radius,
      Shape const &shape, math::Mat3x4f const &shape_transform,
      math::Mat3x4f const &shape_transform_inverse) noexcept;

  friend std::optional<Positionful_contact_geometry>
  particle_shape_positionful_contact_geometry(
      math::Vec3f const &particle_position, float particle_radius,
      Shape const &shape, math::Mat3x4f const &shape_transform,
      math::Mat3x4f const &shape_transform_inverse) noexcept;

  friend std::optional<Positionful_contact_geometry>
  shape_shape_contact_geometry(
      Shape const &shape_a, math::Mat3x4f const &transform_a,
      math::Mat3x4f const &inverse_transform_a, Shape const &shape_b,
      math::Mat3x4f const &transform_b,
      math::Mat3x4f const &inverse_transform_b) noexcept;

private:
  std::variant<Ball, Capsule, Box> _v;
};

inline Aabb bounds(Ball const &ball, math::Vec3f const &position) {
  return {.min = position - math::Vec3f::all(ball.radius),
          .max = position + math::Vec3f::all(ball.radius)};
}

inline Aabb bounds(Capsule const &capsule, math::Vec3f const &position,
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

inline std::optional<Positionless_contact_geometry>
particle_ball_positionless_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius,
    Ball const &ball, math::Vec3f const &ball_position) noexcept {
  auto const displacement = particle_position - ball_position;
  auto const distance2 = length_squared(displacement);
  auto const contact_distance = ball.radius + particle_radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 > contact_distance2) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const distance = std::sqrt(distance2);
    return Positionless_contact_geometry{
        .normal = displacement / distance,
        .separation = distance - contact_distance,
    };
  } else {
    return Positionless_contact_geometry{
        .normal = math::Vec3f::x_axis(),
        .separation = -contact_distance,
    };
  }
}

inline std::optional<Positionless_contact_geometry>
particle_capsule_positionless_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius,
    Capsule const &capsule, math::Vec3f const &capsule_position,
    math::Vec3f const &capsule_axis) noexcept {
  auto const t =
      std::clamp(dot(particle_position - capsule_position, capsule_axis),
                 -capsule.half_height, capsule.half_height);
  auto const p = capsule_position + t * capsule_axis;
  auto const d = particle_position - p;
  auto const distance_squared = length_squared(d);
  auto const contact_distance = particle_radius + capsule.radius;
  auto const contact_distance_squared = contact_distance * contact_distance;
  if (distance_squared > contact_distance_squared) {
    return std::nullopt;
  } else if (distance_squared != 0.0f) {
    auto const distance = std::sqrt(distance_squared);
    return Positionless_contact_geometry{
        .normal = d / distance,
        .separation = distance - contact_distance,
    };
  } else {
    auto const normal =
        cross(std::abs(capsule_axis.x) < std::abs(capsule_axis.y)
                  ? math::Vec3f::x_axis()
                  : math::Vec3f::y_axis(),
              capsule_axis);
    return Positionless_contact_geometry{
        .normal = normal,
        .separation = -contact_distance,
    };
  }
}

inline std::optional<Positionless_contact_geometry>
particle_box_positionless_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius, Box const &box,
    math::Mat3x4f const &box_transform,
    math::Mat3x4f const &box_transform_inverse) noexcept {
  auto const shape_space_particle_position =
      box_transform_inverse * math::Vec4f{particle_position, 1.0f};
  if (std::abs(shape_space_particle_position.x) - particle_radius >
          box.half_extents[0] ||
      std::abs(shape_space_particle_position.y) - particle_radius >
          box.half_extents[1] ||
      std::abs(shape_space_particle_position.z) - particle_radius >
          box.half_extents[2]) {
    return std::nullopt;
  }
  auto const shape_space_clamped_particle_position =
      math::Vec3f{std::clamp(shape_space_particle_position.x,
                             -box.half_extents[0], box.half_extents[0]),
                  std::clamp(shape_space_particle_position.y,
                             -box.half_extents[1], box.half_extents[1]),
                  std::clamp(shape_space_particle_position.z,
                             -box.half_extents[2], box.half_extents[2])};
  auto const displacement =
      shape_space_particle_position - shape_space_clamped_particle_position;
  auto const distance2 = math::length_squared(displacement);
  auto const particle_radius2 = particle_radius * particle_radius;
  if (distance2 > particle_radius2) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const distance = std::sqrt(distance2);
    auto const normal =
        box_transform * math::Vec4f{displacement, 0.0f} / distance;
    auto const separation = distance - particle_radius;
    return Positionless_contact_geometry{.normal = normal,
                                         .separation = separation};
  } else {
    auto const face_distances = std::array<float, 6>{
        shape_space_clamped_particle_position.x + box.half_extents[0],
        box.half_extents[0] - shape_space_clamped_particle_position.x,
        shape_space_clamped_particle_position.y + box.half_extents[1],
        box.half_extents[1] - shape_space_clamped_particle_position.y,
        shape_space_clamped_particle_position.z + box.half_extents[2],
        box.half_extents[2] - shape_space_clamped_particle_position.z};
    auto const face_normals = std::array<math::Vec3f, 6>{
        -column(box_transform, 0), column(box_transform, 0),
        -column(box_transform, 1), column(box_transform, 1),
        -column(box_transform, 2), column(box_transform, 2)};
    auto const face_index =
        std::min_element(face_distances.begin(), face_distances.end()) -
        face_distances.begin();
    auto const separation = -face_distances[face_index] - particle_radius;
    auto const normal = face_normals[face_index];
    return Positionless_contact_geometry{.normal = normal,
                                         .separation = separation};
  }
}

inline std::optional<Positionless_contact_geometry>
particle_shape_positionless_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius,
    Shape const &shape, math::Mat3x4f const &shape_transform,
    math::Mat3x4f const &shape_transform_inverse) noexcept {
  return std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Ball>) {
          return particle_ball_positionless_contact_geometry(
              particle_position, particle_radius, arg,
              column(shape_transform, 3));
        } else if constexpr (std::is_same_v<T, Capsule>) {
          return particle_capsule_positionless_contact_geometry(
              particle_position, particle_radius, arg,
              column(shape_transform, 3), column(shape_transform, 1));
        } else {
          static_assert(std::is_same_v<T, Box>);
          return particle_box_positionless_contact_geometry(
              particle_position, particle_radius, arg, shape_transform,
              shape_transform_inverse);
        }
      },
      shape._v);
}

inline std::optional<Positionful_contact_geometry>
particle_ball_positionful_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius,
    Ball const &ball, math::Vec3f const &ball_position) noexcept {
  auto const displacement = particle_position - ball_position;
  auto const distance2 = math::length_squared(displacement);
  auto const contact_distance = ball.radius + particle_radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 <= contact_distance2) {
    auto const distance = std::sqrt(distance2);
    auto const normal = displacement / distance;
    auto const position =
        ball_position + std::min(ball.radius, distance) * normal;
    return Positionful_contact_geometry{.position = position,
                                        .normal = normal,
                                        .separation =
                                            distance - contact_distance};
  } else {
    return std::nullopt;
  }
}

inline std::optional<Positionful_contact_geometry>
particle_capsule_positionful_contact_geometry(math::Vec3f const &, float,
                                              Capsule const &,
                                              math::Vec3f const &,
                                              math::Vec3f const &) noexcept {
  return std::nullopt;
}

inline std::optional<Positionful_contact_geometry>
particle_box_positionful_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius, Box const &box,
    math::Mat3x4f const &box_transform,
    math::Mat3x4f const &box_transform_inverse) noexcept {
  auto const shape_space_particle_position =
      box_transform_inverse * math::Vec4f{particle_position, 1.0f};
  if (std::abs(shape_space_particle_position.x) - particle_radius >
          box.half_extents[0] ||
      std::abs(shape_space_particle_position.y) - particle_radius >
          box.half_extents[1] ||
      std::abs(shape_space_particle_position.z) - particle_radius >
          box.half_extents[2]) {
    return std::nullopt;
  }
  auto const shape_space_clamped_particle_position =
      math::Vec3f{std::clamp(shape_space_particle_position.x,
                             -box.half_extents[0], box.half_extents[0]),
                  std::clamp(shape_space_particle_position.y,
                             -box.half_extents[1], box.half_extents[1]),
                  std::clamp(shape_space_particle_position.z,
                             -box.half_extents[2], box.half_extents[2])};
  auto const displacement =
      shape_space_particle_position - shape_space_clamped_particle_position;
  auto const distance2 = math::length_squared(displacement);
  auto const particle_radius2 = particle_radius * particle_radius;
  if (distance2 > particle_radius2) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const position =
        box_transform *
        math::Vec4f{shape_space_clamped_particle_position, 1.0f};
    auto const distance = std::sqrt(distance2);
    auto const normal = (particle_position - position) / distance;
    auto const separation = distance - particle_radius;
    return Positionful_contact_geometry{
        .position = position, .normal = normal, .separation = separation};
  } else {
    auto const face_distances = std::array<float, 6>{
        shape_space_clamped_particle_position.x + box.half_extents[0],
        box.half_extents[0] - shape_space_clamped_particle_position.x,
        shape_space_clamped_particle_position.y + box.half_extents[1],
        box.half_extents[1] - shape_space_clamped_particle_position.y,
        shape_space_clamped_particle_position.z + box.half_extents[2],
        box.half_extents[2] - shape_space_clamped_particle_position.z};
    auto const face_index =
        std::min_element(face_distances.begin(), face_distances.end()) -
        face_distances.begin();
    auto const face_normals = std::array<math::Vec3f, 6>{
        -column(box_transform, 0), column(box_transform, 0),
        -column(box_transform, 1), column(box_transform, 1),
        -column(box_transform, 2), column(box_transform, 2)};
    auto const position =
        box_transform *
        math::Vec4f{shape_space_clamped_particle_position, 1.0f};
    auto const normal = face_normals[face_index];
    auto const separation = -face_distances[face_index] - particle_radius;
    return Positionful_contact_geometry{
        .position = position, .normal = normal, .separation = separation};
  }
}

inline std::optional<Positionful_contact_geometry>
particle_shape_positionful_contact_geometry(
    math::Vec3f const &particle_position, float particle_radius,
    Shape const &shape, math::Mat3x4f const &shape_transform,
    math::Mat3x4f const &shape_transform_inverse) noexcept {
  return std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Ball>) {
          return particle_ball_positionful_contact_geometry(
              particle_position, particle_radius, arg,
              column(shape_transform, 3));
        } else if constexpr (std::is_same_v<T, Capsule>) {
          return particle_capsule_positionful_contact_geometry(
              particle_position, particle_radius, arg,
              column(shape_transform, 3), column(shape_transform, 1));
        } else {
          static_assert(std::is_same_v<T, Box>);
          return particle_box_positionful_contact_geometry(
              particle_position, particle_radius, arg, shape_transform,
              shape_transform_inverse);
        }
      },
      shape._v);
}

inline std::optional<Positionful_contact_geometry>
ball_ball_contact_geometry(Ball const &b1, math::Vec3f const &b1_position,
                           Ball const &b2, math::Vec3f const &b2_position) {
  auto const displacement = b1_position - b2_position;
  auto const distance2 = math::length_squared(displacement);
  auto const contact_distance = b1.radius + b2.radius;
  auto const contact_distance2 = contact_distance * contact_distance;
  if (distance2 > contact_distance2) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const distance = std::sqrt(distance2);
    auto const normal = displacement / distance;
    auto const position =
        b1.radius < distance ? b1_position + normal * b1.radius : b2_position;
    return Positionful_contact_geometry{.position = position,
                                        .normal = normal,
                                        .separation =
                                            distance - contact_distance};
  } else {
    return Positionful_contact_geometry{.position = b1_position,
                                        .normal = math::Vec3f{1.0f, 0.0f, 0.0f},
                                        .separation = -contact_distance};
  }
}

inline std::optional<Positionful_contact_geometry>
ball_capsule_contact_geometry(Ball const &, math::Vec3f const &,
                              Capsule const &, math::Vec3f const &,
                              math::Vec3f const &) noexcept {
  return std::nullopt;
}

inline std::optional<Positionful_contact_geometry>
ball_box_contact_geometry(Ball const &ball, math::Vec3f const &ball_position,
                          Box const &box, math::Mat3x4f const &box_transform,
                          math::Mat3x4f const &inverse_box_transform) {
  auto const box_space_ball_position =
      inverse_box_transform * math::Vec4f{ball_position, 1.0f};
  if (std::abs(box_space_ball_position.x) - ball.radius > box.half_extents[0] ||
      std::abs(box_space_ball_position.y) - ball.radius > box.half_extents[1] ||
      std::abs(box_space_ball_position.z) - ball.radius > box.half_extents[2]) {
    return std::nullopt;
  }
  auto const clamped_box_space_ball_position =
      math::Vec3f{std::clamp(box_space_ball_position.x, -box.half_extents[0],
                             box.half_extents[0]),
                  std::clamp(box_space_ball_position.y, -box.half_extents[1],
                             box.half_extents[1]),
                  std::clamp(box_space_ball_position.z, -box.half_extents[2],
                             box.half_extents[2])};
  auto const displacement =
      box_space_ball_position - clamped_box_space_ball_position;
  auto const distance2 = math::length_squared(displacement);
  if (distance2 > ball.radius * ball.radius) {
    return std::nullopt;
  } else if (distance2 != 0.0f) {
    auto const distance = std::sqrt(distance2);
    auto const position =
        box_transform * math::Vec4f{clamped_box_space_ball_position, 1.0f};
    auto const normal = (ball_position - position) / distance;
    return Positionful_contact_geometry{.position = position,
                                        .normal = normal,
                                        .separation = distance - ball.radius};
  } else {
    auto const distances =
        math::Vec3f{box.half_extents[0] - std::abs(box_space_ball_position.x),
                    box.half_extents[1] - std::abs(box_space_ball_position.y),
                    box.half_extents[2] - std::abs(box_space_ball_position.z)};
    auto const axis = distances.x <= distances.y && distances.x <= distances.z
                          ? 0
                      : distances.y <= distances.z ? 1
                                                   : 2;
    auto const normal =
        (std::signbit(box_space_ball_position[axis]) ? -1.0f : 1.0f) *
        math::Vec3f{box_transform[0][axis], box_transform[1][axis],
                    box_transform[2][axis]};
    return Positionful_contact_geometry{.position = ball_position,
                                        .normal = normal,
                                        .separation =
                                            -distances[axis] - ball.radius};
  }
}

inline std::optional<Positionful_contact_geometry>
capsule_capsule_contact_geometry(Capsule const &, math::Vec3f const &,
                                 math::Vec3f const &, Capsule const &,
                                 math::Vec3f const &,
                                 math::Vec3f const &) noexcept {
  return std::nullopt;
}

inline std::optional<Positionful_contact_geometry>
capsule_box_contact_geometry(Capsule const &, math::Vec3f const &,
                             math::Vec3f const &, Box const &,
                             math::Mat3x4f const &) noexcept {
  return std::nullopt;
}

inline std::optional<Positionful_contact_geometry>
box_box_contact_geometry(Box const &b1, math::Mat3x4f const &b1_transform,
                         Box const &b2, math::Mat3x4f const &b2_transform) {
  auto const project_to_axis = [](Box const &b, math::Mat3x4f const &m,
                                  math::Vec3f const &v) {
    return b.half_extents[0] * std::abs(dot(v, column(m, 0))) +
           b.half_extents[1] * std::abs(dot(v, column(m, 1))) +
           b.half_extents[2] * std::abs(dot(v, column(m, 2)));
  };
  auto const center_displacement =
      column(b1_transform, 3) - column(b2_transform, 3);
  auto const separation_on_axis = [&](math::Vec3f const &v) {
    auto const b1_projection = project_to_axis(b1, b1_transform, v);
    auto const b2_projection = project_to_axis(b2, b2_transform, v);
    auto const distance = std::abs(dot(center_displacement, v));
    return distance - (b1_projection + b2_projection);
  };
  auto const edge_edge_contact_position =
      [&](math::Vec3f const &p1, math::Vec3f const &d1, float s1,
          math::Vec3f const &p2, math::Vec3f const &d2, float s2,
          bool use_one) -> math::Vec3f {
    auto const d1_length2 = math::length_squared(d1);
    auto const d2_length2 = math::length_squared(d2);
    auto const d1_dot_d2 = math::dot(d1, d2);
    auto const p_diff = p1 - p2;
    auto const dp_sta_1 = math::dot(d1, p_diff);
    auto const dp_sta_2 = math::dot(d2, p_diff);
    auto const denominator = d1_length2 * d2_length2 - d1_dot_d2 * d1_dot_d2;
    if (std::abs(denominator) < 0.0001f) {
      return use_one ? p1 : p2;
    }
    auto const inverse_denominator = 1.0f / denominator;
    auto const mua =
        (d1_dot_d2 * dp_sta_2 - d2_length2 * dp_sta_1) * inverse_denominator;
    auto const mub =
        (d1_length2 * dp_sta_2 - d1_dot_d2 * dp_sta_1) * inverse_denominator;
    if (std::abs(mua) > s1 || std::abs(mub) > s2) {
      return use_one ? p1 : p2;
    } else {
      auto const c1 = d1 * mua + p1;
      auto const c2 = d2 * mub + p2;
      return 0.5f * (c1 + c2);
    }
  };
  auto separating_axes = std::array<math::Vec3f, 15>{
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
    auto const position =
        b2_transform *
        math::Vec4f{
            dot(separating_axes[3], normal) >= 0.0f ? b2.half_extents[0]
                                                    : -b2.half_extents[0],
            dot(separating_axes[4], normal) >= 0.0f ? b2.half_extents[1]
                                                    : -b2.half_extents[1],
            dot(separating_axes[5], normal) >= 0.0f ? b2.half_extents[2]
                                                    : -b2.half_extents[2],
            1.0f};
    return Positionful_contact_geometry{
        .position = position, .normal = normal, .separation = best_separation};
  } else if (best_separating_axis_index < 6) {
    // vertex of box one on face of box two
    auto const axis = separating_axes[best_separating_axis_index];
    auto const normal = dot(axis, center_displacement) >= 0.0f ? axis : -axis;
    auto const position =
        b1_transform *
        math::Vec4f{
            dot(separating_axes[0], normal) >= 0.0f ? -b1.half_extents[0]
                                                    : b1.half_extents[0],
            dot(separating_axes[1], normal) >= 0.0f ? -b1.half_extents[1]
                                                    : b1.half_extents[1],
            dot(separating_axes[2], normal) >= 0.0f ? -b1.half_extents[2]
                                                    : b1.half_extents[2],
            1.0f};
    return Positionful_contact_geometry{
        .position = position, .normal = normal, .separation = best_separation};
  } else {
    auto const b1_axis_index = (best_separating_axis_index - 6) / 3;
    auto const b2_axis_index = (best_separating_axis_index - 6) % 3;
    auto const axis = separating_axes[best_separating_axis_index];
    auto const normal = dot(axis, center_displacement) < 0.0f ? -axis : axis;
    auto const b1_extents =
        math::Vec3f{b1.half_extents[0], b1.half_extents[1], b1.half_extents[2]};
    auto const on_b1_edge =
        b1_transform *
        math::Vec4f{math::Vec3f{[&](int axis_index) {
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
        math::Vec3f{b2.half_extents[0], b2.half_extents[1], b2.half_extents[2]};
    auto const on_b2_edge =
        b2_transform *
        math::Vec4f{math::Vec3f{[&](int axis_index) {
                      if (axis_index == b2_axis_index) {
                        return 0.0f;
                      } else {
                        return dot(separating_axes[axis_index + 3], normal) <
                                       0.0f
                                   ? -b2_extents[axis_index]
                                   : b2_extents[axis_index];
                      }
                    }},
                    1.0f};
    return Positionful_contact_geometry{
        .position = edge_edge_contact_position(
            on_b1_edge, separating_axes[b2_axis_index],
            b1_extents[b1_axis_index], on_b2_edge,
            separating_axes[b2_axis_index + 3], b2_extents[b2_axis_index],
            best_basis_separating_axis_index >= 3),
        .normal = normal,
        .separation = best_separation};
  }
}

inline std::optional<Positionful_contact_geometry> shape_shape_contact_geometry(
    Shape const &shape_a, math::Mat3x4f const &transform_a,
    math::Mat3x4f const &inverse_transform_a, Shape const &shape_b,
    math::Mat3x4f const &transform_b,
    math::Mat3x4f const &inverse_transform_b) noexcept {
  return std::visit(
      [&](auto &&a) {
        using T = std::decay_t<decltype(a)>;
        if constexpr (std::is_same_v<T, Ball>) {
          return std::visit(
              [&](auto &&b) {
                using U = std::decay_t<decltype(b)>;
                if constexpr (std::is_same_v<U, Ball>) {
                  return ball_ball_contact_geometry(a, column(transform_a, 3),
                                                    b, column(transform_b, 3));
                } else if constexpr (std::is_same_v<U, Capsule>) {
                  return ball_capsule_contact_geometry(
                      a, column(transform_a, 3), b, column(transform_b, 3),
                      column(transform_b, 2));
                } else {
                  static_assert(std::is_same_v<U, Box>);
                  return ball_box_contact_geometry(a, column(transform_a, 3), b,
                                                   transform_b,
                                                   inverse_transform_b);
                }
              },
              shape_b._v);
        } else if constexpr (std::is_same_v<T, Capsule>) {
          return std::visit(
              [&](auto &&b) {
                using U = std::decay_t<decltype(b)>;
                if constexpr (std::is_same_v<U, Ball>) {
                  auto retval = ball_capsule_contact_geometry(
                      b, column(transform_b, 3), a, column(transform_a, 3),
                      column(transform_a, 1));
                  if (retval) {
                    retval->normal = -retval->normal;
                  }
                  return retval;
                } else if constexpr (std::is_same_v<U, Capsule>) {
                  return capsule_capsule_contact_geometry(
                      a, column(transform_a, 3), column(transform_a, 1), b,
                      column(transform_b, 3), column(transform_b, 1));
                } else {
                  static_assert(std::is_same_v<U, Box>);
                  return capsule_box_contact_geometry(a, column(transform_a, 3),
                                                      column(transform_a, 1), b,
                                                      transform_b);
                }
              },
              shape_b._v);
        } else {
          static_assert(std::is_same_v<T, Box>);
          return std::visit(
              [&](auto &&b) {
                using U = std::decay_t<decltype(b)>;
                if constexpr (std::is_same_v<U, Ball>) {
                  auto retval = ball_box_contact_geometry(
                      b, column(transform_b, 3), a, transform_a,
                      inverse_transform_a);
                  if (retval) {
                    retval->normal = -retval->normal;
                  }
                  return retval;
                } else if constexpr (std::is_same_v<U, Capsule>) {
                  auto retval = capsule_box_contact_geometry(
                      b, column(transform_b, 3), column(transform_b, 1), a,
                      transform_a);
                  if (retval) {
                    retval->normal = -retval->normal;
                  }
                  return retval;
                } else {
                  static_assert(std::is_same_v<U, Box>);
                  return box_box_contact_geometry(a, transform_a, b,
                                                  transform_b);
                }
              },
              shape_b._v);
        }
      },
      shape_a._v);
}
} // namespace physics
} // namespace marlon

#endif