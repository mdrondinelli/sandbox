#ifndef MARLON_GRAPHICS_GL_WIREFRAME_MESH_H
#define MARLON_GRAPHICS_GL_WIREFRAME_MESH_H

#include <cstdint>

#include "../wireframe_mesh.h"
#include "wrappers/unique_buffer.h"
#include "wrappers/unique_vertex_array.h"

namespace marlon {
namespace graphics {
namespace gl {
class Wireframe_mesh final : public graphics::Wireframe_mesh {
public:
  explicit Wireframe_mesh(Wireframe_mesh_create_info const &create_info);

  void bind_vertex_array() const noexcept;

  void draw() const noexcept;

private:
  std::uint32_t _index_count;
  wrappers::Unique_buffer _index_buffer;
  wrappers::Unique_buffer _vertex_buffer;
  wrappers::Unique_vertex_array _vertex_array;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif