#ifndef MARLON_GRAPHICS_GL_DEFAULT_RENDER_TARGET_H
#define MARLON_GRAPHICS_GL_DEFAULT_RENDER_TARGET_H

#include "render_target.h"
#include "window.h"

namespace marlon {
namespace graphics {
namespace gl {
struct Default_render_target_create_info {
  Window *window;
};

class Default_render_target final : public Render_target {
public:
  Default_render_target() noexcept = default;

  explicit Default_render_target(Default_render_target_create_info const &create_info) noexcept;

  math::Vec2i get_extents() const noexcept final;

  std::uint32_t get_framebuffer() const noexcept final;

  void on_render() noexcept final;

private:
  Window *_window{nullptr};
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
