#ifndef MARLON_GRAPHICS_GL_UNIQUE_BUFFER_H
#define MARLON_GRAPHICS_GL_UNIQUE_BUFFER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
class Unique_buffer {
public:
  constexpr Unique_buffer() noexcept : _handle{0} {}

  constexpr explicit Unique_buffer(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Unique_buffer();

  Unique_buffer(Unique_buffer &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Unique_buffer &operator=(Unique_buffer &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Unique_buffer &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Unique_buffer make_unique_buffer();
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif