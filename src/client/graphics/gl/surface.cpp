#include "surface.h"

namespace marlon {
namespace rendering {
Gl_surface::Impl::Impl(Surface_create_info const &create_info) noexcept
    : _material{static_cast<Gl_material *>(create_info.material)},
      _mesh{static_cast<Gl_mesh *>(create_info.mesh)} {}

Gl_surface::Gl_surface(Surface_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon