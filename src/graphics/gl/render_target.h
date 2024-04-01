#ifndef MARLON_GRAPHICS_GL_RENDER_DESTINATION_H
#define MARLON_GRAPHICS_GL_RENDER_DESTINATION_H

#include <cstdint>

#include "../../math/vec.h"
#include "../render_target.h"

namespace marlon {
namespace graphics {
class Gl_render_target : public Render_target {
public:
  virtual ~Gl_render_target() = default;

  virtual math::Vec2i get_extents() const noexcept = 0;

  virtual std::uint32_t get_framebuffer() const noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif