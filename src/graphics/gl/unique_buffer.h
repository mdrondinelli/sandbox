#ifndef MARLON_RENDERING_GL_UNIQUE_BUFFER_H
#define MARLON_RENDERING_GL_UNIQUE_BUFFER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace rendering {
class Gl_unique_buffer {
public:
  constexpr Gl_unique_buffer() noexcept : _handle{0} {}

  constexpr explicit Gl_unique_buffer(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Gl_unique_buffer();

  Gl_unique_buffer(Gl_unique_buffer &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_buffer &operator=(Gl_unique_buffer &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Gl_unique_buffer &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Gl_unique_buffer make_gl_unique_buffer();
} // namespace rendering
} // namespace marlon

#endif