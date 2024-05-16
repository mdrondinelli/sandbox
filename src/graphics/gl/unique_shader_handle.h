#ifndef MARLON_GRAPHICS_GL_UNIQUE_SHADER_H
#define MARLON_GRAPHICS_GL_UNIQUE_SHADER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
class Unique_shader_handle {
public:
  Unique_shader_handle() noexcept : _handle{0} {}

  explicit Unique_shader_handle(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Unique_shader_handle();

  Unique_shader_handle(Unique_shader_handle &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Unique_shader_handle &operator=(Unique_shader_handle &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Unique_shader_handle &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Unique_shader_handle make_unique_shader(std::uint32_t type);
}
} // namespace graphics
} // namespace marlon

#endif