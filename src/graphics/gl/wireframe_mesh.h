#ifndef MARLON_GRAPHICS_GL_WIREFRAME_MESH_H
#define MARLON_GRAPHICS_GL_WIREFRAME_MESH_H

#include <cstdint>

#include "../wireframe_mesh.h"
#include "unique_buffer_handle.h"
#include "unique_vertex_array_handle.h"

namespace marlon {
namespace graphics {
class Gl_wireframe_mesh : public Wireframe_mesh {
public:
  explicit Gl_wireframe_mesh(Wireframe_mesh_create_info const &create_info);

  void bind_vertex_array() const noexcept;

  void draw() const noexcept;

private:
  std::uint32_t _index_count;
  Gl_unique_buffer_handle _index_buffer;
  Gl_unique_buffer_handle _vertex_buffer;
  Gl_unique_vertex_array_handle _vertex_array;
};
} // namespace graphics
} // namespace marlon

#endif