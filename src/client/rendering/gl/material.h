#ifndef MARLON_RENDERING_GL_MATERIAL_H
#define MARLON_RENDERING_GL_MATERIAL_H

#include "../material.h"

namespace marlon {
namespace rendering {
class Gl_material : public Material {
public:
  class Impl {
  public:
    explicit Impl(Material_create_info const &create_info) noexcept;

  private:
    Rgb_spectrum _albedo;
  };

  explicit Gl_material(Material_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif