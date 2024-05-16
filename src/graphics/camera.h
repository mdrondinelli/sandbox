#ifndef MARLON_GRAPHICS_CAMERA_H
#define MARLON_GRAPHICS_CAMERA_H

#include "../math/math.h"

namespace marlon {
namespace graphics {
struct Camera {
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec2f zoom{math::Vec2f::all(1.0f)};
  float near_plane_distance{0.1f};
  float cascaded_shadow_map_distance{1000.0f};
  int cascaded_shadow_map_slices{4};
  int cascaded_shadow_map_resolution{1024};
  float exposure{0.02f};
};
} // namespace graphics
} // namespace marlon

#endif