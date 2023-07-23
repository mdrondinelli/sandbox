#ifndef MARLONR_GRAPHICS_GL_MESH_H
#define MARLONR_GRAPHICS_GL_MESH_H

#include <cstdint>

#include "../mesh.h"
#include "unique_buffer.h"
#include "unique_vertex_array.h"

namespace marlon {
namespace graphics {
class Gl_mesh : public Mesh {
  friend class Gl_scene;

public:
  class Impl {
  public:
    explicit Impl(Mesh_create_info const &create_info);

    void bind_vertex_array() const noexcept;

    void draw() const noexcept;

  private:
    std::uint32_t _index_count;
    Mesh_index_format _index_format;
    Gl_unique_buffer _index_buffer;
    Gl_unique_buffer _vertex_buffer;
    Gl_unique_vertex_array _vertex_array;
  };

  explicit Gl_mesh(Mesh_create_info const &create_info);

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif