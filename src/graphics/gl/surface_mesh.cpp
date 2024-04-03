#include "surface_mesh.h"

#include <tuple>

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
Gl_surface_mesh::Impl::Impl(Surface_mesh_create_info const &create_info)
    : _index_count{static_cast<std::uint32_t>(create_info.indices.size())},
      _index_buffer{gl_make_unique_buffer()},
      _vertex_buffer{gl_make_unique_buffer()},
      _vertex_array{gl_make_unique_vertex_array()} {
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
  glVertexArrayAttribFormat(_vertex_array.get(),
                            0,
                            3,
                            GL_FLOAT,
                            GL_FALSE,
                            offsetof(Surface_vertex, position));
  glVertexArrayAttribFormat(_vertex_array.get(),
                            1,
                            2,
                            GL_FLOAT,
                            GL_FALSE,
                            offsetof(Surface_vertex, texcoord));
  glVertexArrayAttribBinding(_vertex_array.get(), 0, 0);
  glVertexArrayAttribBinding(_vertex_array.get(), 1, 0);
}

void Gl_surface_mesh::Impl::bind_vertex_array() const noexcept {
  glBindVertexArray(_vertex_array.get());
}

void Gl_surface_mesh::Impl::draw() const noexcept {
  glDrawElements(GL_TRIANGLES, _index_count, GL_UNSIGNED_SHORT, nullptr);
}

Gl_surface_mesh::Gl_surface_mesh(Surface_mesh_create_info const &create_info)
    : _impl{create_info} {}
} // namespace graphics
} // namespace marlon