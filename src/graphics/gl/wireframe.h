#ifndef MARLON_GRAPHICS_GL_WIREFRAME_H
#define MARLON_GRAPHICS_GL_WIREFRAME_H

#include <math/mat.h>

#include "../rgb_spectrum.h"
#include "../wireframe.h"
#include "wireframe_mesh.h"

namespace marlon {
namespace graphics {
namespace gl {
class Wireframe final : public graphics::Wireframe {
public:
  explicit Wireframe(Wireframe_create_info const &create_info) noexcept
      : _mesh{static_cast<Wireframe_mesh *>(create_info.mesh)},
        _color{create_info.color},
        _transform{create_info.transform} {}

  Wireframe_mesh *get_mesh() const noexcept { return _mesh; }

  Rgb_spectrum const &get_color() const noexcept { return _color; }

  math::Mat3x4f const &get_transform() const noexcept final {
    return _transform;
  }

  void set_transform(math::Mat3x4f const &transform) noexcept final {
    _transform = transform;
  }

private:
  Wireframe_mesh *_mesh;
  Rgb_spectrum _color;
  math::Mat3x4f _transform;
};
}
} // namespace graphics
} // namespace marlon

#endif