#include "unique_vertex_array.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_vertex_array::~Gl_unique_vertex_array() { glDeleteVertexArrays(1, &_handle); }

Gl_unique_vertex_array make_gl_unique_vertex_array() {
  GLuint handle;
  glCreateVertexArrays(1, &handle);
  return Gl_unique_vertex_array{handle};
}
} // namespace rendering
} // namespace marlon