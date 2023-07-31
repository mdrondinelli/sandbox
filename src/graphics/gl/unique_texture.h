#ifndef MARLON_GRAPHICS_GL_UNIQUE_TEXTURE_H
#define MARLON_GRAPHICS_GL_UNIQUE_TEXTURE_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
class Gl_unique_texture_handle {
public:
  Gl_unique_texture_handle() noexcept : _handle{} {}

  explicit Gl_unique_texture_handle(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Gl_unique_texture_handle();

  Gl_unique_texture_handle(Gl_unique_texture_handle &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_texture_handle &
  operator=(Gl_unique_texture_handle &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

private:
  void swap(Gl_unique_texture_handle &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Gl_unique_texture_handle gl_make_unique_texture(std::uint32_t target);
} // namespace graphics
} // namespace marlon

#endif