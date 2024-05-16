#ifndef MARLON_GRAPHICS_WIREFRAME_H
#define MARLON_GRAPHICS_WIREFRAME_H

#include "../math/mat.h"
#include "rgb_spectrum.h"
#include "wireframe_mesh.h"

namespace marlon {
namespace graphics {
struct Wireframe {
  Wireframe_mesh const *mesh{};
  Rgb_spectrum color{Rgb_spectrum::black()};
  math::Mat3x4f transform{math::Mat3x4f::identity()};
  bool visible{true};
};
} // namespace graphics
} // namespace marlon

#endif