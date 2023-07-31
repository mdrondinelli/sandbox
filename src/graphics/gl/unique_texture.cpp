#include "unique_texture.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_texture::~Gl_unique_texture() { glDeleteTextures(1, &_handle); }

Gl_unique_texture make_gl_unique_texture(std::uint32_t target) {
  GLuint handle;
  glCreateTextures(static_cast<GLenum>(target), 1, &handle);
  return Gl_unique_texture{static_cast<std::uint32_t>(handle)};
}
} // namespace graphics
} // namespace marlon