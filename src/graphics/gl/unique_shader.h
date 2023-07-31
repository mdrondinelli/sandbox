#ifndef MARLON_GRAPHICS_GL_UNIQUE_SHADER_H
#define MARLON_GRAPHICS_GL_UNIQUE_SHADER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
class Gl_unique_shader_handle {
public:
  Gl_unique_shader_handle() noexcept : _handle{0} {}

  explicit Gl_unique_shader_handle(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Gl_unique_shader_handle();

  Gl_unique_shader_handle(Gl_unique_shader_handle &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_shader_handle &operator=(Gl_unique_shader_handle &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Gl_unique_shader_handle &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Gl_unique_shader_handle gl_make_unique_shader(std::uint32_t type);
} // namespace graphics
} // namespace marlon

#endif