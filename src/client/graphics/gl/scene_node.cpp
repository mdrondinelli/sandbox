#include "scene_node.h"

namespace marlon {
namespace rendering {
Gl_scene_node::Impl::Impl(Scene_node_create_info const &create_info) noexcept
    : _translation{create_info.translation}, _rotation{create_info.rotation},
      _scale{create_info.scale} {}

Gl_scene_node::Gl_scene_node(Scene_node_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon