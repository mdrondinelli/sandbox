#include "unique_shader_program.h"

#include <glad/gl.h>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
Unique_shader_program::~Unique_shader_program() { glDeleteProgram(_handle); }

Unique_shader_program make_unique_shader_program() {
  return Unique_shader_program{glCreateProgram()};
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon