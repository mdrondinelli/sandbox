#ifndef MARLON_GRAPHICS_GL_WINDOW_H
#define MARLON_GRAPHICS_GL_WINDOW_H

#include "../../math/vec.h"

namespace marlon {
namespace graphics {
namespace gl {
class Window {
public:
  virtual ~Window() {}

  virtual math::Vec2i get_framebuffer_extents() const noexcept = 0;
};
}
} // namespace graphics
} // namespace marlon

#endif