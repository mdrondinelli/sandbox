#include "default_render_target.h"

namespace marlon {
namespace graphics {
Gl_default_render_target::Gl_default_render_target(
    Gl_default_render_target_create_info const &create_info) noexcept
    : _window{create_info.window} {}

math::Vec2i Gl_default_render_target::get_extents() const noexcept {
  return _window->get_framebuffer_extents();
}

std::uint32_t Gl_default_render_target::get_framebuffer() const noexcept {
  return 0;
}
} // namespace graphics
} // namespace marlon