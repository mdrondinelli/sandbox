#ifndef MARLON_GRAPHICS_SURFACE_MATERIAL_H
#define MARLON_GRAPHICS_SURFACE_MATERIAL_H

#include "rgb_spectrum.h"
#include "texture.h"

namespace marlon {
namespace graphics {
struct Surface_material {
  Texture *base_color_texture{};
  Rgb_spectrum base_color_tint{1.0f, 1.0f, 1.0f};
};
} // namespace graphics
} // namespace marlon

#endif