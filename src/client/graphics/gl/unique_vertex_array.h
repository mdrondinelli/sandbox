#ifndef MARLON_RENDERING_GL_UNIQUE_VERTEX_ARRAY_H
#define MARLON_RENDERING_GL_UNIQUE_VERTEX_ARRAY_H

#include <cstdint>

#include <utility>

namespace marlon {
namespace rendering {
class Gl_unique_vertex_array {
public:
  constexpr Gl_unique_vertex_array() noexcept : _handle{0} {}

  constexpr explicit Gl_unique_vertex_array(std::uint32_t handle) noexcept
      : _handle{handle} {}

  ~Gl_unique_vertex_array();

  Gl_unique_vertex_array(Gl_unique_vertex_array &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_unique_vertex_array &operator=(Gl_unique_vertex_array &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
  }

  std::uint32_t get() const noexcept { return _handle; }

private:
  void swap(Gl_unique_vertex_array &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};

Gl_unique_vertex_array make_gl_unique_vertex_array();
} // namespace rendering
} // namespace marlon

#endif