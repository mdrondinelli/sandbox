#ifndef MARLON_GRAPHICS_WIREFRAME_H
#define MARLON_GRAPHICS_WIREFRAME_H

#include "../math/mat.h"
#include "rgb_spectrum.h"
#include "wireframe_mesh.h"

namespace marlon {
namespace graphics {
struct Wireframe_create_info {
  Wireframe_mesh *mesh;
  Rgb_spectrum color{Rgb_spectrum::black()};
  math::Mat3x4f transform{math::Mat3x4f::identity()};
};

class Wireframe {
public:
  virtual math::Mat3x4f const &get_transform() const noexcept = 0;

  virtual void set_transform(math::Mat3x4f const &transform) noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif