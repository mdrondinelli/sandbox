#ifndef MARLON_GRAPHICS_SURFACE_MESH_H
#define MARLON_GRAPHICS_SURFACE_MESH_H

#include <span>

#include "../math/vec.h"

namespace marlon {
namespace graphics {
struct Surface_vertex {
  math::Vec3f position;
  math::Vec3f normal;
  math::Vec2f texcoord;
};

struct Surface_mesh_create_info {
  std::span<std::uint16_t const> indices;
  std::span<Surface_vertex const> vertices;
};

class Surface_mesh {};
} // namespace graphics
} // namespace marlon

#endif