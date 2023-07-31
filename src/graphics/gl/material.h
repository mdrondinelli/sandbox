#ifndef MARLON_GRAPHICS_GL_MATERIAL_H
#define MARLON_GRAPHICS_GL_MATERIAL_H

#include "../material.h"
#include "texture.h"

namespace marlon {
namespace graphics {
class Gl_material : public Material {
  friend class Gl_scene;

public:
  class Impl {
  public:
    explicit Impl(Material_create_info const &create_info) noexcept;

    Gl_texture *get_base_color_texture() const noexcept {
      return _base_color_texture;
    }

    Rgb_spectrum const &get_base_color_tint() const noexcept {
      return _base_color_tint;
    }

  private:
    Gl_texture *_base_color_texture;
    Rgb_spectrum _base_color_tint;
  };

  explicit Gl_material(Material_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace graphics
} // namespace marlon

#endif