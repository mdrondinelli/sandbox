#ifndef MARLON_GRAPHICS_SURFACE_H
#define MARLON_GRAPHICS_SURFACE_H

#include "../math/mat.h"

#include "surface_material.h"
#include "surface_mesh.h"

namespace marlon {
namespace graphics {
struct Surface {
  Surface_mesh const *mesh{};
  Surface_material material{};
  math::Mat3x4f transform{math::Mat3x4f::identity()};
  bool visible{true};
  bool shadow_casting{true};
};
} // namespace graphics
} // namespace marlon

#endif