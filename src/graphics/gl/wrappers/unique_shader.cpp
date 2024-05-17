#include "unique_shader.h"

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
namespace wrappers {
Unique_shader::~Unique_shader() { glDeleteShader(_handle); }

Unique_shader make_unique_shader(std::uint32_t type) {
  return Unique_shader{glCreateShader(type)};
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon