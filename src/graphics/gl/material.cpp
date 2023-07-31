#include "material.h"

namespace marlon {
namespace graphics {
Gl_material::Impl::Impl(Material_create_info const &create_info) noexcept
    : _albedo{create_info.base_color} {}

Gl_material::Gl_material(Material_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace graphics
} // namespace marlon