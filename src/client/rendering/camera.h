#ifndef MARLON_RENDERING_CAMERA_H
#define MARLON_RENDERING_CAMERA_H

namespace marlon {
namespace rendering {
struct Camera_create_info {
  float near_plane_distance;
  float far_plane_distance;
  float aspect_ratio;
  float vertical_fov;
};

class Camera {
public:
  virtual ~Camera() = default;

  virtual float near_plane_distance() const noexcept = 0;

  virtual void near_plane_distance(float new_distance) noexcept = 0;

  virtual float far_plane_distance() const noexcept = 0;

  virtual void far_plane_distance(float new_distance) = 0;

  virtual float aspect_ratio() const noexcept = 0;

  virtual void aspect_ratio(float new_aspect_ratio) noexcept = 0;

  virtual float vertical_fov() const noexcept = 0;

  virtual void vertical_fov(float new_vertical_fov) noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif