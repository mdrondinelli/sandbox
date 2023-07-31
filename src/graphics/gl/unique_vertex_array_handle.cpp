#include "unique_vertex_array_handle.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_vertex_array_handle::~Gl_unique_vertex_array_handle() { glDeleteVertexArrays(1, &_handle); }

Gl_unique_vertex_array_handle gl_make_unique_vertex_array() {
  GLuint handle;
  glCreateVertexArrays(1, &handle);
  return Gl_unique_vertex_array_handle{handle};
}
} // namespace rendering
} // namespace marlon