#ifndef MARLON_GRAPHICS_WIREFRAME_MESH_H
#define MARLON_GRAPHICS_WIREFRAME_MESH_H

#include <span>

#include "../math/vec.h"

namespace marlon {
namespace graphics {
struct Wireframe_mesh_create_info {
  std::span<std::uint16_t const> indices;
  std::span<math::Vec3f const> vertices;
};

class Wireframe_mesh {};
} // namespace graphics
} // namespace marlon

#endif