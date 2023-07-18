#include "surface_instance.h"

namespace marlon {
namespace rendering {
Gl_surface_instance::Impl::Impl(
    Surface_instance_create_info const &create_info) noexcept
    : _surface{static_cast<Gl_surface *>(create_info.surface)},
      _scene_node{static_cast<Gl_scene_node *>(create_info.scene_node)} {}

Gl_surface_instance::Gl_surface_instance(
    Surface_instance_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon