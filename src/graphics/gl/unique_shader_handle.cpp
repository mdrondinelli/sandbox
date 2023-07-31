#include "unique_shader_handle.h"

#include <glad/glad.h>

namespace marlon {
namespace graphics {
Gl_unique_shader_handle::~Gl_unique_shader_handle() { glDeleteShader(_handle); }

Gl_unique_shader_handle gl_make_unique_shader(std::uint32_t type) {
  return Gl_unique_shader_handle{glCreateShader(type)};
}
} // namespace graphics
} // namespace marlon