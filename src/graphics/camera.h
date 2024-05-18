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
  int csm_cascade_count{1};
  int csm_cascade_resolution{4096};
  float csm_distance{10.0f};
  float exposure{0.02f};
};
} // namespace graphics
} // namespace marlon

#endif