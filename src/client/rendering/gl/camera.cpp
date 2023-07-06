#include "camera.h"

namespace marlon {
namespace rendering {
Gl_camera::Gl_camera(Camera_create_info const &create_info) noexcept
    : _near_plane_distance{create_info.near_plane_distance},
      _far_plane_distance{create_info.far_plane_distance},
      _aspect_ratio{create_info.aspect_ratio},
      _vertical_fov{create_info.vertical_fov} {}

float Gl_camera::near_plane_distance() const noexcept {
  return _near_plane_distance;
}

void Gl_camera::near_plane_distance(float new_distance) noexcept {
  _near_plane_distance = new_distance;
}

float Gl_camera::far_plane_distance() const noexcept {
  return _far_plane_distance;
}

void Gl_camera::far_plane_distance(float new_distance) noexcept {
  _far_plane_distance = new_distance;
}

float Gl_camera::aspect_ratio() const noexcept { return _aspect_ratio; }

void Gl_camera::aspect_ratio(float new_aspect_ratio) noexcept {
  _aspect_ratio = new_aspect_ratio;
}

float Gl_camera::vertical_fov() const noexcept { return _vertical_fov; }

void Gl_camera::vertical_fov(float new_vertical_fov) noexcept {
  _vertical_fov = new_vertical_fov;
}
} // namespace rendering
} // namespace marlon