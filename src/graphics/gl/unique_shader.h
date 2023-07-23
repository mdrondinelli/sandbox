#ifndef MARLON_GRAPHICS_GL_UNIQUE_SHADER_H
#define MARLON_GRAPHICS_GL_UNIQUE_SHADER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
class Gl_unique_shader {
public:
  Gl_unique_shader() noexcept : _handle{0} {}

  explicit Gl_unique_shader(std::uint32_t handle) noexcept : _handle{handle} {}

  ~Gl_unique_shader();

  Gl_unique_shader(Gl_unique_shader &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_shader &operator=(Gl_unique_shader &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Gl_unique_shader &other) noexcept { std::swap(_handle, other._handle); }

  std::uint32_t _handle;
};

Gl_unique_shader make_gl_unique_shader(std::uint32_t type);
} // namespace rendering
} // namespace marlon

#endif