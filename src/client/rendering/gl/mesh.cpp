#include "mesh.h"

#include <tuple>

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_mesh::Gl_mesh(Mesh_create_info const &create_info)
    : _index_count{create_info.index_count},
      _index_format{create_info.index_format},
      _index_buffer{make_gl_unique_buffer()},
      _vertex_buffer{make_gl_unique_buffer()},
      _vertex_array{make_gl_unique_vertex_array()} {
  const auto index_size = [&]() {
    switch (create_info.index_format) {
    case Mesh_index_format::uint16:
      return sizeof(std::uint16_t);
    case Mesh_index_format::uint32:
      return sizeof(std::uint32_t);
    }
    throw;
  }();
  glNamedBufferStorage(_index_buffer.handle(),
                       index_size * create_info.index_count,
                       create_info.index_data, 0);
  glVertexArrayElementBuffer(_vertex_array.handle(), _index_buffer.handle());
  glNamedBufferStorage(_vertex_buffer.handle(),
                       create_info.vertex_format.stride *
                           create_info.vertex_count,
                       create_info.vertex_data, 0);
  glVertexArrayVertexBuffer(
      _vertex_array.handle(), 0, _vertex_buffer.handle(), 0,
      static_cast<GLsizei>(create_info.vertex_format.stride));
  const auto [position_size, position_type] = [&]() {
    switch (create_info.vertex_format.position_fetch_info.format) {
    case Mesh_vertex_position_format::float3:
      return std::tuple{3, GL_FLOAT};
    }
    throw;
  }();
  glVertexArrayAttribFormat(
      _vertex_array.handle(), 0, position_size, position_type, GL_FALSE,
      create_info.vertex_format.position_fetch_info.offset);
  glVertexArrayAttribBinding(_vertex_array.handle(), 0, 0);
}
} // namespace rendering
} // namespace marlon