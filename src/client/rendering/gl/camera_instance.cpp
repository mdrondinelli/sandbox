#include "camera_instance.h"

namespace marlon {
namespace rendering {
Gl_camera_instance::Gl_camera_instance(
    Camera_instance_create_info const &create_info) noexcept
    : _camera{static_cast<Gl_camera *>(create_info.camera)},
      _node{static_cast<Gl_scene_node *>(create_info.node)} {}

Gl_camera *Gl_camera_instance::camera() const noexcept { return _camera; }

Gl_scene_node *Gl_camera_instance::node() const noexcept { return _node; }
} // namespace rendering
} // namespace marlon