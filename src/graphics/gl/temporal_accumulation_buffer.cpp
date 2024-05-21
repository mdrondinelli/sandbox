#include "temporal_accumulation_buffer.h"

#include <iostream>
#include <stdexcept>

#include <glad/gl.h>

namespace marlon::graphics::gl {
Temporal_accumulation_buffer::Temporal_accumulation_buffer(
    Temporal_accumulation_buffer_create_info const &create_info)
    : _texture{wrappers::make_unique_texture(GL_TEXTURE_2D)},
      _framebuffer{wrappers::make_unique_framebuffer()} {
  glTextureStorage2D(_texture.get(),
                     1,
                     GL_RGB16F,
                     create_info.extents.x,
                     create_info.extents.y);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTextureParameteri(_texture.get(), GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTextureParameteri(_texture.get(), GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glNamedFramebufferTexture(
      _framebuffer.get(), GL_COLOR_ATTACHMENT0, _texture.get(), 0);
  if (glCheckNamedFramebufferStatus(_framebuffer.get(), GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error{"Failed to make a complete framebuffer for "
                             "temporal accumulation buffer"};
  }
}
} // namespace marlon::graphics::gl