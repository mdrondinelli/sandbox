#include "unique_buffer_handle.h"

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
Unique_buffer_handle::~Unique_buffer_handle() {
  glDeleteBuffers(1, &_handle);
}

Unique_buffer_handle make_unique_buffer() {
  GLuint handle;
  glCreateBuffers(1, &handle);
  return Unique_buffer_handle{handle};
}
}
} // namespace graphics
} // namespace marlon