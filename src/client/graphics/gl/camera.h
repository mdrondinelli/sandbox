#ifndef MARLON_RENDERING_GL_CAMERA_H
#define MARLON_RENDERING_GL_CAMERA_H

#include "../../../shared/math/mat.h"
#include "../camera.h"

namespace marlon {
namespace rendering {
class Gl_camera : public Camera {
  friend class Gl_graphics;

public:
  class Impl {
  public:
    explicit Impl(Camera_create_info const &create_info) noexcept;

    math::Mat4x4f calculate_clip_matrix() const noexcept {
      return math::Mat4x4f{{_zoom_x, 0.0f, 0.0f, 0.0f},
                           {0.0f, _zoom_y, 0.0f, 0.0f},
                           {0.0f, 0.0f,
                            -(_far_plane_distance + _near_plane_distance) /
                                (_far_plane_distance - _near_plane_distance),
                            -2.0f * _near_plane_distance * _far_plane_distance /
                                (_far_plane_distance - _near_plane_distance)},
                           {0.0f, 0.0f, -1.0f, 0.0f}};
    }

  private:
    float _near_plane_distance;
    float _far_plane_distance;
    float _zoom_x;
    float _zoom_y;
  };

  explicit Gl_camera(Camera_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif