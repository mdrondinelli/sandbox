#ifndef MARLON_RENDERING_GL_VERTEX_ARRAY_H
#define MARLON_RENDERING_GL_VERTEX_ARRAY_H

#include <cstdint>

#include <utility>

#include "default_handle_init.h"

namespace marlon {
namespace rendering {
class Gl_vertex_array {
public:
  Gl_vertex_array() noexcept : _handle{0} {}

  explicit Gl_vertex_array(Gl_default_handle_init);

  ~Gl_vertex_array();

  Gl_vertex_array(Gl_vertex_array &&other) noexcept
      : _handle{std::exchange(other._handle, 0)} {}

  Gl_vertex_array &operator=(Gl_vertex_array &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
  }

  std::uint32_t handle() const noexcept { return _handle; }

private:
  void swap(Gl_vertex_array &other) noexcept {
    std::swap(_handle, other._handle);
  }

  std::uint32_t _handle;
};
} // namespace rendering
} // namespace marlon

#endif