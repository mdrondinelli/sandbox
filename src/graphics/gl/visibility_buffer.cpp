#include "visibility_buffer.h"

#include <stdexcept>

#include <glad/gl.h>

namespace marlon::graphics::gl {
Visibility_buffer::Visibility_buffer(
    Visibility_buffer_create_info const &create_info)
    : _extents{create_info.extents},
      _depth_texture{wrappers::make_unique_texture(GL_TEXTURE_2D)},
      _color_texture{wrappers::make_unique_texture(GL_TEXTURE_2D)},
      _framebuffer{wrappers::make_unique_framebuffer()} {
  glTextureStorage2D(_depth_texture.get(),
                     1,
                     GL_DEPTH_COMPONENT32F,
                     create_info.extents.x,
                     create_info.extents.y);
  glTextureStorage2D(_color_texture.get(),
                     1,
                     GL_RGB16F,
                     create_info.extents.x,
                     create_info.extents.y);
  glTextureParameteri(_depth_texture.get(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTextureParameteri(_depth_texture.get(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTextureParameteri(_color_texture.get(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTextureParameteri(_color_texture.get(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glNamedFramebufferTexture(
      _framebuffer.get(), GL_DEPTH_ATTACHMENT, _depth_texture.get(), 0);
  glNamedFramebufferTexture(
      _framebuffer.get(), GL_COLOR_ATTACHMENT0, _color_texture.get(), 0);
  if (glCheckNamedFramebufferStatus(_framebuffer.get(), GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error{
        "Failed to make a complete framebuffer for visibility buffer"};
  }
}
} // namespace marlon::graphics::gl