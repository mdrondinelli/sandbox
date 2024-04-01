#ifndef MARLON_ENGINE_CAMERA_H
#define MARLON_ENGINE_CAMERA_H

#include "../math/math.h"

namespace marlon {
namespace engine {
struct Camera_create_info {
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec2f zoom{math::Vec2f::all(1.0f)};
  float near_plane_distance{0.01f};
  float far_plane_distance{1000.0f};
};

class Camera {
public:
  Camera(Camera_create_info const &create_info = {}) noexcept
      : _position{create_info.position},
        _orientation{create_info.orientation},
        _zoom{create_info.zoom},
        _near_plane_distance{create_info.near_plane_distance},
        _far_plane_distance{create_info.far_plane_distance} {}

  Camera(Camera const &other) = delete;

  Camera &operator=(Camera const &other) = delete;

  math::Vec3f const &get_position() const noexcept { return _position; }

  void set_position(math::Vec3f const &position) noexcept {
    _position = position;
  }

  math::Quatf const &get_orientation() const noexcept { return _orientation; }

  void set_orientation(math::Quatf const &orientation) noexcept {
    _orientation = orientation;
  }

  math::Vec2f const &get_zoom() const noexcept { return _zoom; }

  void set_zoom(math::Vec2f const &zoom) noexcept { _zoom = zoom; }

  float get_near_plane_distance() const noexcept {
    return _near_plane_distance;
  }

  void set_near_plane_distance(float near_plane_distance) noexcept {
    _near_plane_distance = near_plane_distance;
  }

  float get_far_plane_distance() const noexcept { return _far_plane_distance; }

  void set_far_plane_distance(float far_plane_distance) noexcept {
    _far_plane_distance = far_plane_distance;
  }

private:
  math::Vec3f _position;
  math::Quatf _orientation;
  math::Vec2f _zoom;
  float _near_plane_distance;
  float _far_plane_distance;
};
} // namespace engine
} // namespace marlon

#endif