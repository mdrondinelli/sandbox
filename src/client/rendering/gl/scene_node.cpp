#include "scene_node.h"

namespace marlon {
namespace rendering {
Gl_scene_node::Gl_scene_node(Scene_node_create_info const &create_info) noexcept
    : _position{create_info.position}, _orientation{create_info.orientation},
      _scale_factor{create_info.scale_factor} {}

math::Vec3f Gl_scene_node::position() const noexcept { return _position; }

Gl_scene_node &
Gl_scene_node::position(math::Vec3f const &new_position) noexcept {
  _position = new_position;
  return *this;
}

Gl_scene_node &
Gl_scene_node::translate(math::Vec3f const &translation) noexcept {
  _position += translation;
  return *this;
}

math::Quatf Gl_scene_node::orientation() const noexcept { return _orientation; }

Gl_scene_node &
Gl_scene_node::orientation(math::Quatf const &new_orientation) noexcept {
  _orientation = new_orientation;
  return *this;
}

Gl_scene_node &Gl_scene_node::pre_rotate(math::Quatf const &rotation) noexcept {
  _orientation = rotation * _orientation;
  return *this;
}

Gl_scene_node &
Gl_scene_node::post_rotate(math::Quatf const &rotation) noexcept {
  _orientation *= rotation;
  return *this;
}

float Gl_scene_node::scale_factor() const noexcept { return _scale_factor; }

Gl_scene_node &Gl_scene_node::scale_factor(float new_scale_factor) noexcept {
  _scale_factor = new_scale_factor;
  return *this;
}

Gl_scene_node &Gl_scene_node::scale(float amount) noexcept {
  _scale_factor *= amount;
  return *this;
}
} // namespace rendering
} // namespace marlon