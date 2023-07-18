#ifndef MARLON_RENDERING_GL_DEFAULT_RENDER_TARGET_H
#define MARLON_RENDERING_GL_DEFAULT_RENDER_TARGET_H

#include "render_target.h"

namespace marlon {
namespace graphics {
class Gl_default_render_target : public Gl_render_target {
public:
  std::uint32_t get_framebuffer() const noexcept final;
};
} // namespace rendering
} // namespace marlon

#endif