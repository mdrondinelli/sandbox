#include "shader.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_shader::Gl_shader(std::uint32_t type) : _handle{glCreateShader(type)} {}
} // namespace rendering
} // namespace marlon