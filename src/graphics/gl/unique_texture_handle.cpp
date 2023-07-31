#include "unique_texture_handle.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_texture_handle::~Gl_unique_texture_handle() { glDeleteTextures(1, &_handle); }

Gl_unique_texture_handle gl_make_unique_texture(std::uint32_t target) {
  GLuint handle;
  glCreateTextures(static_cast<GLenum>(target), 1, &handle);
  return Gl_unique_texture_handle{static_cast<std::uint32_t>(handle)};
}
} // namespace graphics
} // namespace marlon