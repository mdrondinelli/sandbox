#include "vertex_array.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_vertex_array::Gl_vertex_array(Gl_default_handle_init) {
  glCreateVertexArrays(1, &_handle);
}

Gl_vertex_array::~Gl_vertex_array() { glDeleteVertexArrays(1, &_handle); }
} // namespace rendering
} // namespace marlon