#ifndef MARLON_GRAPHICS_GL_UNIQUE_VERTEX_ARRAY_H
#define MARLON_GRAPHICS_GL_UNIQUE_VERTEX_ARRAY_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
  class Unique_vertex_array_handle {
  public:
    constexpr Unique_vertex_array_handle() noexcept : _handle{0} {}

    constexpr explicit Unique_vertex_array_handle(
        std::uint32_t handle) noexcept
        : _handle{handle} {}

    ~Unique_vertex_array_handle();

    Unique_vertex_array_handle(Unique_vertex_array_handle &&other) noexcept
        : _handle{std::exchange(other._handle, 0)} {}

    Unique_vertex_array_handle &
    operator=(Unique_vertex_array_handle &&other) noexcept {
      auto temp{std::move(other)};
      swap(temp);
      return *this;
    }

    std::uint32_t get() const noexcept { return _handle; }

  private:
    void swap(Unique_vertex_array_handle &other) noexcept {
      std::swap(_handle, other._handle);
    }

    std::uint32_t _handle;
  };

  Unique_vertex_array_handle make_unique_vertex_array();
}
} // namespace graphics
} // namespace marlon

#endif