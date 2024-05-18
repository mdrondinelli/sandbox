#include "cascaded_shadow_map.h"

#include <iostream>
#include <stdexcept>

#include <glad/gl.h>

namespace marlon::graphics::gl {
Cascaded_shadow_map::Cascade::Cascade(int resolution)
    : _texture{wrappers::make_unique_texture(GL_TEXTURE_2D)},
      _framebuffer{wrappers::make_unique_framebuffer()} {
  glTextureStorage2D(
      _texture.get(), 1, GL_DEPTH_COMPONENT32F, resolution, resolution);
  glTextureParameteri(
      _texture.get(), GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glNamedFramebufferTexture(
      _framebuffer.get(), GL_DEPTH_ATTACHMENT, _texture.get(), 0);
  // glNamedFramebufferReadBuffer(_framebuffer.get(), GL_NONE);
  // glNamedFramebufferDrawBuffer(_framebuffer.get(), GL_NONE);
  if (glCheckNamedFramebufferStatus(_framebuffer.get(), GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error{
        "Failed to make a complete framebuffer for CSM cascade"};
  }
}

Cascaded_shadow_map::Cascaded_shadow_map(
    Cascaded_shadow_map_create_info const &create_info)
    : _cascade_resolution{create_info.cascade_resolution} {
  for (auto i = 0; i != create_info.cascade_count; ++i) {
    _cascades.emplace_back(_cascade_resolution);
  }
}
} // namespace marlon::graphics::gl