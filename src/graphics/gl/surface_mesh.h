#ifndef MARLONR_GRAPHICS_GL_SURFACE_MESH_H
#define MARLONR_GRAPHICS_GL_SURFACE_MESH_H

#include <cstdint>

#include <math/aabb.h>

#include "../surface_mesh.h"
#include "./wrappers/unique_buffer.h"
#include "./wrappers/unique_vertex_array.h"

namespace marlon::graphics::gl {
class Surface_mesh final : public graphics::Surface_mesh {
public:
  explicit Surface_mesh(Surface_mesh_create_info const &create_info);

  math::Aabb3f const &model_space_bounds() const noexcept {
    return _model_space_bounds;
  }

  void bind_vertex_array() const noexcept;

  void draw() const noexcept;

private:
  std::uint32_t _index_count;
  wrappers::Unique_buffer _index_buffer;
  wrappers::Unique_buffer _vertex_buffer;
  wrappers::Unique_vertex_array _vertex_array;
  math::Aabb3f _model_space_bounds;
};
} // namespace marlon::graphics::gl

#endif