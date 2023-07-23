#include "unique_shader_program.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_shader_program::~Gl_unique_shader_program() {
  glDeleteProgram(_handle);
}

Gl_unique_shader_program make_gl_unique_shader_program() {
  return Gl_unique_shader_program{glCreateProgram()};
}
} // namespace rendering
} // namespace marlon