#include "default_render_target.h"

namespace marlon {
namespace graphics {
namespace gl {
Default_render_target::Default_render_target(
    Default_render_target_create_info const &create_info) noexcept
    : _window{create_info.window} {}

math::Vec2i Default_render_target::get_extents() const noexcept {
  return _window->get_framebuffer_extents();
}

std::uint32_t Default_render_target::get_framebuffer() const noexcept {
  return 0;
}
} // namespace gl
} // namespace graphics
} // namespace marlon