#include "wireframe_mesh.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

namespace marlon {
namespace graphics {
namespace gl {
Wireframe_mesh::Wireframe_mesh(
    Wireframe_mesh_create_info const &create_info)
    : _index_count{static_cast<std::uint32_t>(create_info.indices.size())},
      _index_buffer{make_unique_buffer()},
      _vertex_buffer{make_unique_buffer()},
      _vertex_array{make_unique_vertex_array()} {
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
  glVertexArrayAttribFormat(_vertex_array.get(), 0, 3, GL_FLOAT, GL_FALSE, 0);
  glVertexArrayAttribBinding(_vertex_array.get(), 0, 0);
}

void Wireframe_mesh::bind_vertex_array() const noexcept {
  glBindVertexArray(_vertex_array.get());
}

void Wireframe_mesh::draw() const noexcept {
  glDrawElements(GL_LINES, _index_count, GL_UNSIGNED_SHORT, nullptr);
}
}
} // namespace graphics
} // namespace marlon