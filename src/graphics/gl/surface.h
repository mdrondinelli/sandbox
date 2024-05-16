#ifndef MARLON_GRAPHICS_GL_SURFACE_H
#define MARLON_GRAPHICS_GL_SURFACE_H

#include "../surface.h"
#include "surface_mesh.h"

namespace marlon {
namespace graphics {
namespace gl {
class Surface final : public graphics::Surface {
public:
  explicit Surface(Surface_create_info const &create_info) noexcept
      : _mesh{static_cast<Surface_mesh const *>(create_info.mesh)},
        _material{create_info.material},
        _transform{create_info.transform} {}

  Surface_mesh const *get_mesh() const noexcept final { return _mesh; }

  Surface_material const &get_material() const noexcept final {
    return _material;
  }

  math::Mat3x4f const &get_transform() const noexcept final {
    return _transform;
  }

  void set_transform(math::Mat3x4f const &transform) noexcept final {
    _transform = transform;
  }

private:
  Surface_mesh const *_mesh;
  Surface_material _material;
  math::Mat3x4f _transform;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif