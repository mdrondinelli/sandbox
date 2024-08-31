#ifndef MARLON_GRAPHICS_SURFACE_MATERIAL_H
#define MARLON_GRAPHICS_SURFACE_MATERIAL_H

#include "rgb_spectrum.h"

namespace marlon {
namespace graphics {
struct Surface_material {
  Rgb_spectrum base_color_tint{1.0f, 1.0f, 1.0f};
};
} // namespace graphics
} // namespace marlon

#endif
