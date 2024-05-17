#include "unique_framebuffer.h"

#include <glad/gl.h>

namespace marlon::graphics::gl::wrappers {
Unique_framebuffer::~Unique_framebuffer() {
  glDeleteFramebuffers(1, &_handle);
}

Unique_framebuffer make_unique_framebuffer() {
  std::uint32_t handle;
  glCreateFramebuffers(1, &handle);
  return Unique_framebuffer{handle};
}
} // namespace marlon::graphics::gl::wrappers