#ifndef MARLON_GRAPHICS_GL_MATERIAL_H
#define MARLON_GRAPHICS_GL_MATERIAL_H

#include "../surface_material.h"
#include "texture.h"

namespace marlon {
namespace graphics {
class Gl_surface_material final : public Surface_material {
  friend class Gl_scene;

public:
  class Impl {
  public:
    explicit Impl(Surface_material_create_info const &create_info) noexcept;

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

  explicit Gl_surface_material(
      Surface_material_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace graphics
} // namespace marlon

#endif