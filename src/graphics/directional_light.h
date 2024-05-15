#ifndef MARLON_GRAPHICS_DIRECTIONAL_LIGHT_H
#define MARLON_GRAPHICS_DIRECTIONAL_LIGHT_H

#include "../math/vec.h"
#include "rgb_spectrum.h"

namespace marlon {
namespace graphics {
struct Directional_light {
  Rgb_spectrum irradiance{Rgb_spectrum::white()};
  math::Vec3f direction{math::Vec3f::y_axis()};
};
} // namespace graphics
} // namespace marlon

#endif