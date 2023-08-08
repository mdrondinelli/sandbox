#include "unique_buffer_handle.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#pragma clang diagnostic pop

namespace marlon {
namespace graphics {
Gl_unique_buffer_handle::~Gl_unique_buffer_handle() {
  glDeleteBuffers(1, &_handle);
}

Gl_unique_buffer_handle gl_make_unique_buffer() {
  GLuint handle;
  glCreateBuffers(1, &handle);
  return Gl_unique_buffer_handle{handle};
}
} // namespace graphics
} // namespace marlon