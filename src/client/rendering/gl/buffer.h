#ifndef MARLON_RENDERING_GL_BUFFER_H
#define MARLON_RENDERING_GL_BUFFER_H

#include <cstdint>

#include <utility>

#include "default_handle_init.h"

namespace marlon {
namespace rendering {
class Gl_buffer {
public:
  Gl_buffer() noexcept : _handle{0} {}

  explicit Gl_buffer(Gl_default_handle_init);

  ~Gl_buffer();

  Gl_buffer(Gl_buffer &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_buffer &operator=(Gl_buffer &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  std::uint32_t handle() const noexcept { return _handle; }

private:
  void swap(Gl_buffer &other) noexcept { std::swap(_handle, other._handle); }

  std::uint32_t _handle;
};
} // namespace rendering
} // namespace marlon

#endif