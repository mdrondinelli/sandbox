#ifndef MARLON_RENDERING_GL_CAMERA_H
#define MARLON_RENDERING_GL_CAMERA_H

#include "../camera.h"

namespace marlon {
namespace rendering {
class Gl_camera : public Camera {
public:
  class Impl {
  public:
    explicit Impl(Camera_create_info const &create_info) noexcept;

  private:
    float _near_plane_distance;
    float _far_plane_distance;
    float _aspect_ratio;
    float _vertical_fov;
  };

  explicit Gl_camera(Camera_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif