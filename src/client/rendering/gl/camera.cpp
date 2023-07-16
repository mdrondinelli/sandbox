#include "camera.h"

namespace marlon {
namespace rendering {
Gl_camera::Impl::Impl(Camera_create_info const &create_info) noexcept
    : _near_plane_distance{create_info.near_plane_distance},
      _far_plane_distance{create_info.far_plane_distance},
      _aspect_ratio{create_info.aspect_ratio},
      _vertical_fov{create_info.vertical_fov} {}

Gl_camera::Gl_camera(Camera_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon