#ifndef MARLON_GRAPHICS_GL_UNIQUE_TEXTURE_H
#define MARLON_GRAPHICS_GL_UNIQUE_TEXTURE_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
class Unique_texture_handle {
public:
  constexpr Unique_texture_handle() noexcept : _handle{} {}

  explicit Unique_texture_handle(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Unique_texture_handle();

  Unique_texture_handle(Unique_texture_handle &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Unique_texture_handle &
  operator=(Unique_texture_handle &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }
  
private:
  void swap(Unique_texture_handle &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Unique_texture_handle make_unique_texture(std::uint32_t target);
}
} // namespace graphics
} // namespace marlon

#endif