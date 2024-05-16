#include "unique_vertex_array_handle.h"

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
Unique_vertex_array_handle::~Unique_vertex_array_handle() {
  glDeleteVertexArrays(1, &_handle);
}

Unique_vertex_array_handle make_unique_vertex_array() {
  GLuint handle;
  glCreateVertexArrays(1, &handle);
  return Unique_vertex_array_handle{handle};
}
}
} // namespace graphics
} // namespace marlon