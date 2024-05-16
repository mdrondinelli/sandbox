#ifndef MARLON_GRAPHICS_GL_UNIQUE_BUFFER_H
#define MARLON_GRAPHICS_GL_UNIQUE_BUFFER_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
  class Unique_buffer_handle {
  public:
    constexpr Unique_buffer_handle() noexcept : _handle{0} {}

    constexpr explicit Unique_buffer_handle(std::uint32_t handle) noexcept
        : _handle{handle} {}

    ~Unique_buffer_handle();

    Unique_buffer_handle(Unique_buffer_handle &&other) noexcept
        : _handle{std::exchange(other._handle, 0)} {}

    Unique_buffer_handle &operator=(Unique_buffer_handle &&other) noexcept {
      auto temp{std::move(other)};
      swap(temp);
      return *this;
    }

    std::uint32_t get() const noexcept { return _handle; }

  private:
    void swap(Unique_buffer_handle &other) noexcept {
      std::swap(_handle, other._handle);
    }

    std::uint32_t _handle;
  };

Unique_buffer_handle make_unique_buffer();
}
} // namespace graphics
} // namespace marlon

#endif