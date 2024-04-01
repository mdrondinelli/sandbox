#include "unique_texture_handle.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

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