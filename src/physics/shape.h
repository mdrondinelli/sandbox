#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <optional>
#include <type_traits>
#include <variant>

#include <math/math.h>

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

struct Narrowphase_result;

class Shape {
public:
  constexpr Shape(Ball const &ball) noexcept
      : _v{ball} {}

  constexpr Shape(Capsule const &capsule) noexcept
      : _v{capsule} {}

  constexpr Shape(Box const &box) noexcept
      : _v{box} {}

  constexpr std::variant<Ball, Capsule, Box> const &get() const noexcept {
    return _v;
  }

  friend math::Aabb3f bounds(Shape const &shape,
                             math::Mat3x4f const &transform) noexcept;

  friend std::optional<Narrowphase_result>
  particle_shape_contact(float particle_radius,
                         math::Vec3f const &particle_position,
                         Shape const &shape,
                         math::Mat3x4f const &shape_transform,
                         math::Mat3x4f const &shape_transform_inv) noexcept;

  friend std::optional<Narrowphase_result>
  shape_shape_contact(Shape const &shape_a,
                      math::Mat3x4f const &transform_a,
                      math::Mat3x4f const &transform_a_inv,
                      Shape const &shape_b,
                      math::Mat3x4f const &transform_b,
                      math::Mat3x4f const &transform_b_inv) noexcept;

private:
  std::variant<Ball, Capsule, Box> _v;
};

inline math::Aabb3f bounds(Ball const &ball, math::Vec3f const &position) {
  return {position - math::Vec3f::all(ball.radius),
          position + math::Vec3f::all(ball.radius)};
}

inline math::Aabb3f bounds(Capsule const &capsule,
                           math::Vec3f const &position,
                           math::Vec3f const &axis) noexcept {
  auto const world_space_half_extents =
      abs(axis) + math::Vec3f::all(capsule.radius);
  return {position - world_space_half_extents,
          position + world_space_half_extents};
}

inline math::Aabb3f bounds(Box const &box,
                           math::Mat3x4f const &transform) noexcept {
  auto const world_space_center = column(transform, 3);
  auto const world_space_half_extents =
      abs(transform) * math::Vec4f{box.half_extents, 0.0f};
  return math::Aabb{world_space_center - world_space_half_extents,
                    world_space_center + world_space_half_extents};
}

inline math::Aabb3f bounds(Shape const &shape,
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
} // namespace physics
} // namespace marlon

#endif
