#ifndef MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_FRAMEBUFFER_H
#define MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_FRAMEBUFFER_H

#include <cstdint>

#include <utility>

namespace marlon::graphics::gl::wrappers {
class Unique_framebuffer {
public:
  constexpr Unique_framebuffer() noexcept = default;

  constexpr explicit Unique_framebuffer(std::uint32_t handle) noexcept : _handle{handle} {}

  ~Unique_framebuffer();

  constexpr Unique_framebuffer(Unique_framebuffer &&other) noexcept : _handle{std::exchange(other._handle, 0)} {}

  Unique_framebuffer &operator=(Unique_framebuffer &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  constexpr std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Unique_framebuffer &other) noexcept { std::swap(_handle, other._handle); }

  std::uint32_t _handle{};
};

Unique_framebuffer make_unique_framebuffer();
} // namespace marlon::graphics::gl::wrappers

#endif
