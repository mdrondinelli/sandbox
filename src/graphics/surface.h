#ifndef MARLONR_GRAPHICS_SURFACE_H
#define MARLONR_GRAPHICS_SURFACE_H

#include "../math/mat.h"
#include "../math/quat.h"
#include "../math/vec.h"
#include "material.h"
#include "mesh.h"

namespace marlon {
namespace graphics {
struct Surface_create_info {
  Mesh *mesh{};
  Material *material{};
  math::Mat3x4f transform{math::Mat3x4f::identity()};
};

class Surface {
public:
  virtual math::Mat3x4f const &get_transform() const noexcept = 0;

  virtual void set_transform(math::Mat3x4f const &transform) noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif