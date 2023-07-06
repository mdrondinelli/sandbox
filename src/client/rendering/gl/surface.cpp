#include "surface.h"

namespace marlon {
namespace rendering {
Gl_surface::Gl_surface(Surface_create_info const &create_info) noexcept
    : _mesh{static_cast<Gl_mesh *>(create_info.mesh)},
      _material{static_cast<Gl_material *>(create_info.material)} {}

Gl_mesh *Gl_surface::mesh() const noexcept { return _mesh; }

Gl_material *Gl_surface::material() const noexcept { return _material; }
} // namespace rendering
} // namespace marlon