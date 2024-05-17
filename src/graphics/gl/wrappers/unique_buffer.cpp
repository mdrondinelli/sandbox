#include "unique_buffer.h"

#include <glad/gl.h>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
Unique_buffer::~Unique_buffer() { glDeleteBuffers(1, &_handle); }

Unique_buffer make_unique_buffer() {
  GLuint handle;
  glCreateBuffers(1, &handle);
  return Unique_buffer{handle};
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon