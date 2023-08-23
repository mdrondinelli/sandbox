#ifndef MARLON_GRAPHICS_GL_CAMERA_H
#define MARLON_GRAPHICS_GL_CAMERA_H

#include "../../math/mat.h"
#include "../camera.h"

namespace marlon {
namespace graphics {
class Gl_camera final : public Camera {
public:
  explicit Gl_camera(Camera_create_info const &create_info) noexcept
      : _zoom{create_info.zoom},
        _near_plane_distance{create_info.near_plane_distance},
        _far_plane_distance{create_info.far_plane_distance},
        _position{create_info.position}, _orientation{create_info.orientation} {
  }

  math::Mat3x4f get_view_matrix() const noexcept {
    auto const upper_left_inv =
        math::Mat3x3f{{(1.0f - 2.0f * _orientation.v.y * _orientation.v.y -
                        2.0f * _orientation.v.z * _orientation.v.z),
                       (2.0f * _orientation.v.x * _orientation.v.y +
                        2.0f * _orientation.w * _orientation.v.z),
                       (2.0f * _orientation.v.x * _orientation.v.z -
                        2.0f * _orientation.w * _orientation.v.y)},
                      {(2.0f * _orientation.v.x * _orientation.v.y -
                        2.0f * _orientation.w * _orientation.v.z),
                       (1.0f - 2.0f * _orientation.v.x * _orientation.v.x -
                        2.0f * _orientation.v.z * _orientation.v.z),
                       (2.0f * _orientation.v.y * _orientation.v.z +
                        2.0f * _orientation.w * _orientation.v.x)},
                      {(2.0f * _orientation.v.x * _orientation.v.z +
                        2.0f * _orientation.w * _orientation.v.y),
                       (2.0f * _orientation.v.y * _orientation.v.z -
                        2.0f * _orientation.w * _orientation.v.x),
                       (1.0f - 2.0f * _orientation.v.x * _orientation.v.x -
                        2.0f * _orientation.v.y * _orientation.v.y)}};
    return math::Mat3x4f{
        {upper_left_inv[0][0], upper_left_inv[0][1], upper_left_inv[0][2],
         -(upper_left_inv[0] * _position)},
        {upper_left_inv[1][0], upper_left_inv[1][1], upper_left_inv[1][2],
         -(upper_left_inv[1] * _position)},
        {upper_left_inv[2][0], upper_left_inv[2][1], upper_left_inv[2][2],
         -(upper_left_inv[2] * _position)}};
  }

  math::Mat4x4f get_clip_matrix() const noexcept {
    return math::Mat4x4f{{_zoom.x, 0.0f, 0.0f, 0.0f},
                         {0.0f, _zoom.y, 0.0f, 0.0f},
                         {0.0f, 0.0f,
                          -(_far_plane_distance + _near_plane_distance) /
                              (_far_plane_distance - _near_plane_distance),
                          -2.0f * _near_plane_distance * _far_plane_distance /
                              (_far_plane_distance - _near_plane_distance)},
                         {0.0f, 0.0f, -1.0f, 0.0f}};
  }

  void set_zoom(math::Vec2f const &zoom) noexcept final { _zoom = zoom; }

  void set_near_plane_distance(float d) noexcept final {
    _near_plane_distance = d;
  }

  void set_far_plane_distance(float d) noexcept final {
    _far_plane_distance = d;
  }

  void set_position(math::Vec3f const &position) noexcept {
    _position = position;
  }

  void set_orientation(math::Quatf const &orientation) noexcept {
    _orientation = orientation;
  }

private:
  math::Vec2f _zoom;
  float _near_plane_distance;
  float _far_plane_distance;
  math::Vec3f _position;
  math::Quatf _orientation;
};
} // namespace graphics
} // namespace marlon

#endif