#include "surface_mesh.h"

#include <glad/gl.h>

#include <util/size.h>

namespace marlon::graphics::gl {
using namespace math;
using namespace util;
Surface_mesh::Surface_mesh(Surface_mesh_create_info const &create_info)
    : _index_count{static_cast<std::uint32_t>(create_info.indices.size())},
      _index_buffer{wrappers::make_unique_buffer()},
      _vertex_buffer{wrappers::make_unique_buffer()},
      _vertex_array{wrappers::make_unique_vertex_array()} {
  glNamedBufferStorage(_index_buffer.get(),
                       create_info.indices.size_bytes(),
                       create_info.indices.data(),
                       0);
  glVertexArrayElementBuffer(_vertex_array.get(), _index_buffer.get());
  glNamedBufferStorage(_vertex_buffer.get(),
                       create_info.vertices.size_bytes(),
                       create_info.vertices.data(),
                       0);
  glVertexArrayVertexBuffer(_vertex_array.get(),
                            0,
                            _vertex_buffer.get(),
                            0,
                            static_cast<GLsizei>(sizeof(
                                decltype(create_info.vertices)::element_type)));
  glEnableVertexArrayAttrib(_vertex_array.get(), 0);
  glEnableVertexArrayAttrib(_vertex_array.get(), 1);
  glEnableVertexArrayAttrib(_vertex_array.get(), 2);
  glVertexArrayAttribFormat(_vertex_array.get(),
                            0,
                            3,
                            GL_FLOAT,
                            GL_FALSE,
                            offsetof(Surface_vertex, position));
  glVertexArrayAttribFormat(_vertex_array.get(),
                            1,
                            3,
                            GL_FLOAT,
                            GL_FALSE,
                            offsetof(Surface_vertex, normal));
  glVertexArrayAttribFormat(_vertex_array.get(),
                            2,
                            2,
                            GL_FLOAT,
                            GL_FALSE,
                            offsetof(Surface_vertex, texcoord));
  glVertexArrayAttribBinding(_vertex_array.get(), 0, 0);
  glVertexArrayAttribBinding(_vertex_array.get(), 1, 0);
  glVertexArrayAttribBinding(_vertex_array.get(), 2, 0);
  _model_space_bounds.min = create_info.vertices[0].position;
  _model_space_bounds.max = create_info.vertices[0].position;
  for (auto const &vertex : create_info.vertices.subspan(1)) {
    _model_space_bounds = merge(_model_space_bounds, vertex.position);
  }
}

void Surface_mesh::bind_vertex_array() const noexcept {
  glBindVertexArray(_vertex_array.get());
}

void Surface_mesh::draw() const noexcept {
  glDrawElements(GL_TRIANGLES, _index_count, GL_UNSIGNED_SHORT, nullptr);
}
} // namespace marlon::graphics::gl