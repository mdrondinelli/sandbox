#ifndef MARLON_PHYSICS_SHAPE_H
#define MARLON_PHYSICS_SHAPE_H

#include <type_traits>
#include <variant>

#include "ball.h"
#include "box.h"

namespace marlon {
namespace physics {
class Shape {
public:
  Shape(Ball const &ball) noexcept : _v{ball} {}

  Shape(Box const &box) noexcept : _v{box} {}

  friend Bounding_box bounds(Shape const &shape,
                             math::Mat3x4f const &shape_transform) noexcept {
    return std::visit(
        [&](auto &&arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, Ball>) {
            return bounds(arg, math::Vec3f{shape_transform[0][3],
                                           shape_transform[1][3],
                                           shape_transform[2][3]});
          } else {
            static_assert(std::is_same_v<T, Box>);
            return bounds(arg, shape_transform);
          }
        },
        shape._v);
  }

  friend std::optional<Particle_contact>
  find_particle_contact(math::Vec3f const &particle_position,
                        float particle_radius, Shape const &shape,
                        math::Mat3x4f const &shape_transform,
                        math::Mat3x4f const &shape_transform_inverse) noexcept {
    return std::visit(
        [&](auto &&arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, Ball>) {
            return find_particle_contact(
                particle_position, particle_radius, arg,
                math::Vec3f{shape_transform[0][3], shape_transform[1][3],
                            shape_transform[2][3]});
          } else {
            static_assert(std::is_same_v<T, Box>);
            return find_particle_contact(particle_position, particle_radius,
                                         arg, shape_transform,
                                         shape_transform_inverse);
          }
        },
        shape._v);
  }

private:
  std::variant<Ball, Box> _v;
};
} // namespace physics
} // namespace marlon

#endif