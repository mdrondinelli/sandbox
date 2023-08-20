#ifndef MARLON_GRAPHICS_GL_SURFACE_H
#define MARLON_GRAPHICS_GL_SURFACE_H

#include "../surface.h"
#include "mesh.h"

namespace marlon {
namespace graphics {
class Gl_surface final : public Surface {
public:
  explicit Gl_surface(Surface_create_info const &create_info) noexcept
      : _mesh{static_cast<Gl_mesh *>(create_info.mesh)},
        _material{static_cast<Gl_material *>(create_info.material)},
        _transform{create_info.transform} {}

  Gl_mesh *get_mesh() const noexcept { return _mesh; }

  Gl_material *get_material() const noexcept { return _material; }

  math::Mat3x4f const &get_transform() const noexcept final {
    return _transform;
  }

  void set_transform(math::Mat3x4f const &transform) noexcept final {
    _transform = transform;
  }

private:
  Gl_mesh *_mesh;
  Gl_material *_material;
  math::Mat3x4f _transform;
};
} // namespace graphics
} // namespace marlon

#endif