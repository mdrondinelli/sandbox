#ifndef MARLON_GRAPHICS_GL_VISIBILITY_BUFFER_H
#define MARLON_GRAPHICS_GL_VISIBILITY_BUFFER_H

#include <math/vec.h>

#include "./wrappers/wrappers.h"

namespace marlon::graphics::gl {
struct Visibility_buffer_create_info {
  math::Vec2i extents;
};

class Visibility_buffer {
public:
  constexpr Visibility_buffer() noexcept = default;

  Visibility_buffer(Visibility_buffer_create_info const &create_info);

  constexpr operator bool() const noexcept {
    return extents() != math::Vec2i::zero();
  }

  constexpr math::Vec2i const &extents() const noexcept { return _extents; }

  constexpr std::uint32_t depth_texture() const noexcept {
    return _depth_texture.get();
  }

  constexpr std::uint32_t color_texture() const noexcept {
    return _color_texture.get();
  }

  constexpr std::uint32_t normal_texture() const noexcept {
    return _normal_texture.get();
  }

  constexpr std::uint32_t motion_vectors_texture() const noexcept {
    return _motion_vectors_texture.get();
  }

  constexpr std::uint32_t framebuffer() const noexcept {
    return _framebuffer.get();
  }

private:
  math::Vec2i _extents{};
  wrappers::Unique_texture _depth_texture;
  wrappers::Unique_texture _color_texture;
  wrappers::Unique_texture _normal_texture;
  wrappers::Unique_texture _motion_vectors_texture;
  wrappers::Unique_framebuffer _framebuffer;
};
} // namespace marlon::graphics::gl

#endif