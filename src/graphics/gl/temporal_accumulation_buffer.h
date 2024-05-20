#ifndef MARLON_GRAPHICS_GL_TEMPORAL_ACCUMULATION_BUFFER_H
#define MARLON_GRAPHICS_GL_TEMPORAL_ACCUMULATION_BUFFER_H

#include <math/vec.h>

#include "./wrappers/wrappers.h"

namespace marlon::graphics::gl {
struct Temporal_accumulation_buffer_create_info {
  math::Vec2i extents;
};

class Temporal_accumulation_buffer {
public:
  constexpr Temporal_accumulation_buffer() = default;

  explicit Temporal_accumulation_buffer(
      Temporal_accumulation_buffer_create_info const &create_info);

  std::uint32_t texture() const noexcept {
    return _texture.get();
  }

  std::uint32_t framebuffer() const noexcept {
    return _framebuffer.get();
  }

private:
  wrappers::Unique_texture _texture;
  wrappers::Unique_framebuffer _framebuffer;
};
} // namespace marlon::graphics::gl

#endif