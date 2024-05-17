#include "unique_buffer.h"

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