#include "unique_shader_program.h"

#include <iostream>
#include <vector>

#include <glad/gl.h>

#include "unique_shader.h"

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
namespace {
void compile_shader(GLuint shader) {
  GLint status;
  glCompileShader(shader);
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(shader, log_size, nullptr, log.data());
    std::cerr << log.data() << std::endl;
    throw std::runtime_error{log.data()};
  }
}

void link_shader_program(GLuint shader_program) {
  GLint status;
  glLinkProgram(shader_program);
  glGetProgramiv(shader_program, GL_LINK_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size;
    glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetProgramInfoLog(shader_program, log_size, nullptr, log.data());
    std::cerr << log.data() << std::endl;
    throw std::runtime_error{log.data()};
  }
}
} // namespace

Unique_shader_program::~Unique_shader_program() { glDeleteProgram(_handle); }

Unique_shader_program make_unique_shader_program() {
  return Unique_shader_program{glCreateProgram()};
}

wrappers::Unique_shader_program
make_unique_shader_program(char const *vertex_shader_source,
                           char const *fragment_shader_source) {
  auto const vertex_shader{wrappers::make_unique_shader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader.get(), 1, &vertex_shader_source, nullptr);
  compile_shader(vertex_shader.get());
  auto const fragment_shader{wrappers::make_unique_shader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader.get(), 1, &fragment_shader_source, nullptr);
  compile_shader(fragment_shader.get());
  auto result = wrappers::make_unique_shader_program();
  glAttachShader(result.get(), vertex_shader.get());
  glAttachShader(result.get(), fragment_shader.get());
  link_shader_program(result.get());
  glDetachShader(result.get(), vertex_shader.get());
  glDetachShader(result.get(), fragment_shader.get());
  return result;
}
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon