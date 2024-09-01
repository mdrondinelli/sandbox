#ifndef MARLON_GRAPHICS_GL_WINDOW_H
#define MARLON_GRAPHICS_GL_WINDOW_H

#include "../../math/vec.h"

namespace marlon {
namespace graphics {
namespace gl {
class Window {
public:
  virtual ~Window() {}

  virtual void make_context_current() noexcept = 0;

  virtual void swap_buffers() noexcept = 0;

  virtual math::Vec2i get_framebuffer_extents() const noexcept = 0;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
