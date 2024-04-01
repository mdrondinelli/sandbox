#include "unique_shader_handle.h"

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
Gl_unique_shader_handle::~Gl_unique_shader_handle() { glDeleteShader(_handle); }

Gl_unique_shader_handle gl_make_unique_shader(std::uint32_t type) {
  return Gl_unique_shader_handle{glCreateShader(type)};
}
} // namespace graphics
} // namespace marlon