#include "unique_shader.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_unique_shader::~Gl_unique_shader() {
  glDeleteShader(_handle);
}

Gl_unique_shader make_gl_unique_shader(std::uint32_t type) {
  return Gl_unique_shader{glCreateShader(type)};
}
} // namespace rendering
} // namespace marlon