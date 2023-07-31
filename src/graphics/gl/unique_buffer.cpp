#include "unique_buffer.h"

#include <glad/glad.h>

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