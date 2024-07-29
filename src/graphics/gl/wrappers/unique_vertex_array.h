#ifndef MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_VERTEX_ARRAY_H
#define MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_VERTEX_ARRAY_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace graphics {
namespace gl {
namespace wrappers {
class Unique_vertex_array {
public:
  constexpr Unique_vertex_array() noexcept : _handle{0} {}

  constexpr explicit Unique_vertex_array(std::uint32_t handle) noexcept : _handle{handle} {}

  ~Unique_vertex_array();

  constexpr Unique_vertex_array(Unique_vertex_array &&other) noexcept : _handle{std::exchange(other._handle, 0)} {}

  Unique_vertex_array &operator=(Unique_vertex_array &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  constexpr std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Unique_vertex_array &other) noexcept { std::swap(_handle, other._handle); }

  std::uint32_t _handle;
};

Unique_vertex_array make_unique_vertex_array();
} // namespace wrappers
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
