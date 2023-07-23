#include "camera_instance.h"

namespace marlon {
namespace graphics {
Gl_camera_instance::Impl::Impl(
    Camera_instance_create_info const &create_info) noexcept
    : _camera{static_cast<Gl_camera *>(create_info.camera)},
      _scene_node{static_cast<Gl_scene_node *>(create_info.scene_node)} {}

Gl_camera_instance::Gl_camera_instance(
    Camera_instance_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace graphics
} // namespace marlon