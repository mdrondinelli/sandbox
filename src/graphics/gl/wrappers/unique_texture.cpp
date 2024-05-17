#include "unique_texture.h"

#include <glad/gl.h>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
Unique_texture::~Unique_texture() {
  glDeleteTextures(1, &_handle);
}

Unique_texture make_unique_texture(std::uint32_t target) {
  GLuint handle;
  glCreateTextures(static_cast<GLenum>(target), 1, &handle);
  return Unique_texture{static_cast<std::uint32_t>(handle)};
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon