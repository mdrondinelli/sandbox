#include "unique_buffer.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_buffer::~Gl_unique_buffer() { glDeleteBuffers(1, &_handle); }

Gl_unique_buffer make_gl_unique_buffer() {
  GLuint handle;
  glCreateBuffers(1, &handle);
  return Gl_unique_buffer{handle};
}
} // namespace rendering
} // namespace marlon