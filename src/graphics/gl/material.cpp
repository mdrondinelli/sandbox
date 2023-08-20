#include "material.h"

namespace marlon {
namespace graphics {
Gl_material::Impl::Impl(Material_create_info const &create_info) noexcept
    : _base_color_texture{static_cast<Gl_texture *>(
          create_info.base_color_texture)},
      _base_color_tint{create_info.base_color_tint} {}

Gl_material::Gl_material(Material_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace graphics
} // namespace marlon