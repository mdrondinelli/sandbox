#ifndef MARLONR_RENDERING_GL_MESH_H
#define MARLONR_RENDERING_GL_MESH_H

#include <cstdint>

#include "../mesh.h"
#include "unique_buffer.h"
#include "unique_vertex_array.h"

namespace marlon {
namespace rendering {
class Gl_mesh : public Mesh {
  std::uint32_t _index_count;
  Mesh_index_format _index_format;
  Gl_unique_buffer _index_buffer;
  Gl_unique_buffer _vertex_buffer;
  Gl_unique_vertex_array _vertex_array;

public:
  explicit Gl_mesh(Mesh_create_info const &create_info);

  std::uint32_t index_count() const noexcept {
    return _index_count;
  }

  Mesh_index_format index_format() const noexcept {
    return _index_format;
  }

  std::uint32_t vertex_array() const noexcept { return _vertex_array.get(); }
};
} // namespace rendering
} // namespace marlon

#endif