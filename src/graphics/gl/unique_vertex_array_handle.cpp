#include "unique_vertex_array_handle.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#pragma clang diagnostic pop
#else
#include <glad/glad.h>
#endif

namespace marlon {
namespace graphics {
Gl_unique_vertex_array_handle::~Gl_unique_vertex_array_handle() {
  glDeleteVertexArrays(1, &_handle);
}

Gl_unique_vertex_array_handle gl_make_unique_vertex_array() {
  GLuint handle;
  glCreateVertexArrays(1, &handle);
  return Gl_unique_vertex_array_handle{handle};
}
} // namespace graphics
} // namespace marlon