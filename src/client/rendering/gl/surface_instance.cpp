#include "surface_instance.h"

namespace marlon {
namespace rendering {
Gl_surface_instance::Gl_surface_instance(
    Surface_instance_create_info const &create_info) noexcept
    : _surface{static_cast<Gl_surface *>(create_info.surface)},
      _node{static_cast<Gl_scene_node *>(create_info.node)} {}

Gl_surface *Gl_surface_instance::surface() const noexcept { return _surface; }

Gl_scene_node *Gl_surface_instance::node() const noexcept { return _node; }
} // namespace rendering
} // namespace marlon