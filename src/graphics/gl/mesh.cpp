#include "mesh.h"

#include <tuple>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#pragma clang diagnostic pop

namespace marlon {
namespace graphics {
Gl_mesh::Impl::Impl(Mesh_create_info const &create_info)
    : _index_count{create_info.index_count},
      _index_format{create_info.index_format},
      _index_buffer{gl_make_unique_buffer()},
      _vertex_buffer{gl_make_unique_buffer()},
      _vertex_array{gl_make_unique_vertex_array()} {
  const auto index_size = [&]() {
    switch (create_info.index_format) {
    case Mesh_index_format::uint16:
      return sizeof(std::uint16_t);
    case Mesh_index_format::uint32:
      return sizeof(std::uint32_t);
    }
    throw;
  }();
  glNamedBufferStorage(_index_buffer.get(),
                       index_size * create_info.index_count,
                       create_info.index_data, 0);
  glVertexArrayElementBuffer(_vertex_array.get(), _index_buffer.get());
  glNamedBufferStorage(_vertex_buffer.get(),
                       create_info.vertex_format.stride *
                           create_info.vertex_count,
                       create_info.vertex_data, 0);
  glVertexArrayVertexBuffer(
      _vertex_array.get(), 0, _vertex_buffer.get(), 0,
      static_cast<GLsizei>(create_info.vertex_format.stride));
  glEnableVertexArrayAttrib(_vertex_array.get(), 0);
  glEnableVertexArrayAttrib(_vertex_array.get(), 1);
  const auto [position_size, position_type, position_normalized] = [&]() {
    switch (create_info.vertex_format.position_fetch_info.format) {
    case Mesh_vertex_position_format::float3:
      return std::tuple{3, GL_FLOAT, static_cast<GLboolean>(GL_FALSE)};
    }
    throw;
  }();
  const auto [texcoord_size, texcoord_type, texcoord_normalized] = [&]() {
    switch (create_info.vertex_format.texcoord_fetch_info.format) {
    case Mesh_vertex_texcoord_format::float2:
      return std::tuple{2, GL_FLOAT, static_cast<GLboolean>(GL_FALSE)};
    }
    throw;
  }();
  glVertexArrayAttribFormat(
      _vertex_array.get(), 0, position_size, position_type, position_normalized,
      create_info.vertex_format.position_fetch_info.offset);
  glVertexArrayAttribFormat(
      _vertex_array.get(), 1, texcoord_size, texcoord_type, texcoord_normalized,
      create_info.vertex_format.texcoord_fetch_info.offset);
  glVertexArrayAttribBinding(_vertex_array.get(), 0, 0);
  glVertexArrayAttribBinding(_vertex_array.get(), 1, 0);
}

void Gl_mesh::Impl::bind_vertex_array() const noexcept {
  glBindVertexArray(_vertex_array.get());
}

void Gl_mesh::Impl::draw() const noexcept {
  auto const index_type = [this]() {
    switch (_index_format) {
    case Mesh_index_format::uint16:
      return GL_UNSIGNED_SHORT;
    case Mesh_index_format::uint32:
      return GL_UNSIGNED_INT;
    }
    throw;
  }();
  glDrawElements(GL_TRIANGLES, _index_count, index_type, nullptr);
}

Gl_mesh::Gl_mesh(Mesh_create_info const &create_info) : _impl{create_info} {}
} // namespace graphics
} // namespace marlon