#ifndef MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_TEXTURE_H
#define MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_TEXTURE_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
class Unique_texture {
public:
  constexpr Unique_texture() noexcept : _handle{} {}

  constexpr explicit Unique_texture(std::uint32_t handle) noexcept : _handle{handle} {}

  ~Unique_texture();

  constexpr Unique_texture(Unique_texture &&other) noexcept : _handle{std::exchange(other._handle, 0)} {}

  Unique_texture &operator=(Unique_texture &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  constexpr std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Unique_texture &other) noexcept { std::swap(_handle, other._handle); }

  std::uint32_t _handle;
};

Unique_texture make_unique_texture(std::uint32_t target);
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
