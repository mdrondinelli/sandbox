#ifndef MARLON_GRAPHICS_GL_DEFAULT_RENDER_TARGET_H
#define MARLON_GRAPHICS_GL_DEFAULT_RENDER_TARGET_H

#include "render_target.h"
#include "window.h"

namespace marlon {
namespace graphics {
struct Gl_default_render_target_create_info {
  Gl_window const *window;
};

class Gl_default_render_target : public Gl_render_target {
public:
  explicit Gl_default_render_target(
      Gl_default_render_target_create_info const &create_info) noexcept;

  math::Vec2i get_extents() const noexcept final;

  std::uint32_t get_framebuffer() const noexcept final;

private:
  Gl_window const *_window;
};
} // namespace graphics
} // namespace marlon

#endif