#ifndef MARLON_RENDERING_GL_CAMERA_H
#define MARLON_RENDERING_GL_CAMERA_H

#include "../camera.h"

namespace marlon {
namespace rendering {
class Gl_camera : public Camera {
public:
  explicit Gl_camera(Camera_create_info const &create_info) noexcept;

  float near_plane_distance() const noexcept override;

  void near_plane_distance(float new_distance) noexcept override;

  float far_plane_distance() const noexcept override;

  void far_plane_distance(float new_distance) noexcept override;

  float aspect_ratio() const noexcept override;

  void aspect_ratio(float new_aspect_ratio) noexcept override;

  float vertical_fov() const noexcept override;

  void vertical_fov(float new_vertical_fov) noexcept override;

private:
  float _near_plane_distance;
  float _far_plane_distance;
  float _aspect_ratio;
  float _vertical_fov;
};
} // namespace rendering
} // namespace marlon

#endif