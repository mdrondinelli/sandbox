#include "unique_vertex_array.h"

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
Unique_vertex_array::~Unique_vertex_array() {
  glDeleteVertexArrays(1, &_handle);
}

Unique_vertex_array make_unique_vertex_array() {
  GLuint handle;
  glCreateVertexArrays(1, &handle);
  return Unique_vertex_array{handle};
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon