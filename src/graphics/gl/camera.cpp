#include "camera.h"

namespace marlon {
namespace graphics {
Gl_camera::Impl::Impl(Camera_create_info const &create_info) noexcept
    : _near_plane_distance{create_info.near_plane_distance},
      _far_plane_distance{create_info.far_plane_distance},
      _zoom_x{create_info.zoom_x},
      _zoom_y{create_info.zoom_y} {}

Gl_camera::Gl_camera(Camera_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace graphics
} // namespace marlon