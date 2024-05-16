#ifndef MARLON_GRAPHICS_SURFACE_H
#define MARLON_GRAPHICS_SURFACE_H

#include "../math/mat.h"

#include "surface_material.h"
#include "surface_mesh.h"

namespace marlon {
namespace graphics {
struct Surface_create_info {
  Surface_mesh const *mesh;
  Surface_material material;
  math::Mat3x4f transform{math::Mat3x4f::identity()};
};

class Surface {
public:
  virtual Surface_mesh const *get_mesh() const noexcept = 0;

  virtual Surface_material const &get_material() const noexcept = 0;

  virtual math::Mat3x4f const &get_transform() const noexcept = 0;

  virtual void set_transform(math::Mat3x4f const &transform) noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif