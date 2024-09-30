#ifndef MARLON_GRAPHICS_CAMERA_H
#define MARLON_GRAPHICS_CAMERA_H

#include <math/quat.h>
#include <math/vec.h>

namespace marlon {
namespace graphics {
struct Camera {
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec2f zoom{math::Vec2f::all(1.0f)};
  float near_plane_distance{0.1f};
  int csm_texture_resolution{4096};
  int csm_cascade_count{4};
  float csm_render_distance{128.0f};
  float exposure{0.01f};
};
} // namespace graphics
} // namespace marlon

#endif
