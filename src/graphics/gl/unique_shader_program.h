#ifndef MARLON_RENDERING_GL_UNIQUE_SHADER_PROGRAM_H
#define MARLON_RENDERING_GL_UNIQUE_SHADER_PROGRAM_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
class Gl_unique_shader_program {
public:
  constexpr Gl_unique_shader_program() noexcept : _handle{0} {}

  constexpr explicit Gl_unique_shader_program(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Gl_unique_shader_program();

  Gl_unique_shader_program(Gl_unique_shader_program &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_shader_program &
  operator=(Gl_unique_shader_program &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept {
    return _handle;
  }

private:
  void swap(Gl_unique_shader_program &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Gl_unique_shader_program make_gl_unique_shader_program();
} // namespace rendering
} // namespace marlon

#endif