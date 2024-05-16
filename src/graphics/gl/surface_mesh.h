#ifndef MARLONR_GRAPHICS_GL_SURFACE_MESH_H
#define MARLONR_GRAPHICS_GL_SURFACE_MESH_H

#include <cstdint>

#include "../surface_mesh.h"
#include "unique_buffer_handle.h"
#include "unique_vertex_array_handle.h"

namespace marlon {
namespace graphics {
namespace gl {
  class Surface_mesh final : public graphics::Surface_mesh {
  public:
    explicit Surface_mesh(Surface_mesh_create_info const &create_info);

    void bind_vertex_array() const noexcept;

    void draw() const noexcept;

  private:
    std::uint32_t _index_count;
    Unique_buffer_handle _index_buffer;
    Unique_buffer_handle _vertex_buffer;
    Unique_vertex_array_handle _vertex_array;
  };
}
} // namespace graphics
} // namespace marlon

#endif