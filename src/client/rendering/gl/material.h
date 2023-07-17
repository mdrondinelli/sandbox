#ifndef MARLON_RENDERING_GL_MATERIAL_H
#define MARLON_RENDERING_GL_MATERIAL_H

#include "../material.h"

namespace marlon {
namespace rendering {
class Gl_material : public Material {
  friend class Gl_scene;

public:
  class Impl {
  public:
    explicit Impl(Material_create_info const &create_info) noexcept;

    Rgb_spectrum const &get_albedo() const noexcept {
      return _albedo;
    }

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