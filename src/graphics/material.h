#ifndef MARLON_RENDERING_MATERIAL_H
#define MARLON_RENDERING_MATERIAL_H

#include "rgb_spectrum.h"

namespace marlon {
namespace graphics {
struct Material_create_info {
  Rgb_spectrum albedo;
};

class Material {};
} // namespace rendering
} // namespace marlon

#endif