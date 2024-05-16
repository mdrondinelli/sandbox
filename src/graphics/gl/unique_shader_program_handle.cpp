#include "unique_shader_program_handle.h"

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
Unique_shader_program_handle::~Unique_shader_program_handle() {
  glDeleteProgram(_handle);
}

Unique_shader_program_handle make_unique_shader_program() {
  return Unique_shader_program_handle{glCreateProgram()};
}
}
} // namespace graphics
} // namespace marlon