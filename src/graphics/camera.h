#ifndef MARLON_GRAPHICS_CAMERA_H
#define MARLON_GRAPHICS_CAMERA_H

#include "../math/quat.h"
#include "../math/vec.h"

namespace marlon {
namespace graphics {
struct Camera_create_info {
  math::Vec2f zoom{1.0f, 1.0f};
  float near_plane_distance{0.001f};
  float far_plane_distance{1000.0f};
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
};

class Camera {
public:
  virtual void set_zoom(math::Vec2f const &zoom) noexcept = 0;

  virtual void set_near_plane_distance(float d) noexcept = 0;

  virtual void set_far_plane_distance(float d) noexcept = 0;

  virtual void set_position(math::Vec3f const &position) noexcept = 0;

  virtual void set_orientation(math::Quatf const &orientation) noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif