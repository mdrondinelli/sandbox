#include "material.h"

namespace marlon {
namespace rendering {
Gl_material::Gl_material(Material_create_info const &create_info) noexcept
    : _albedo{create_info.albedo} {}

Rgb_spectrum Gl_material::albedo() const noexcept { return _albedo; }
} // namespace rendering
} // namespace marlon