#ifndef MARLON_GRAPHICS_GL_RENDER_DESTINATION_H
#define MARLON_GRAPHICS_GL_RENDER_DESTINATION_H

#include <cstdint>

#include <math/vec.h>

#include "../render_target.h"

namespace marlon {
namespace graphics {
namespace gl {
class Render_target : public graphics::Render_target {
public:
  virtual ~Render_target() = default;

  virtual math::Vec2i get_extents() const noexcept = 0;

  virtual std::uint32_t get_framebuffer() const noexcept = 0;

  virtual void on_render() noexcept = 0;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
