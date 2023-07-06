#ifndef MARLON_RENDERING_GL_MATERIAL_H
#define MARLON_RENDERING_GL_MATERIAL_H

#include "../material.h"

namespace marlon {
namespace rendering {
class Gl_material : public Material {
public:
  explicit Gl_material(Material_create_info const &create_info) noexcept;

  Rgb_spectrum albedo() const noexcept override;

private:
  Rgb_spectrum _albedo;
};
} // namespace rendering
} // namespace marlon

#endif