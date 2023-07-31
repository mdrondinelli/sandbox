#ifndef MARLON_GRAPHICS_MESH_H
#define MARLON_GRAPHICS_MESH_H

#include <cstdint>

namespace marlon {
namespace graphics {
enum class Mesh_index_format { uint16, uint32 };

enum class Mesh_vertex_position_format { float3 };

enum class Mesh_vertex_texcoord_format { float2 };

struct Mesh_vertex_position_fetch_info {
  Mesh_vertex_position_format format;
  std::uint32_t offset;
};

struct Mesh_vertex_texcoord_fetch_info {
  Mesh_vertex_texcoord_format format;
  std::uint32_t offset;
};

struct Mesh_vertex_format {
  Mesh_vertex_position_fetch_info position_fetch_info;
  Mesh_vertex_texcoord_fetch_info texcoord_fetch_info;
  // std::optional<Mesh_vertex_attribute_fetch_info> normal_fetch_info;
  // std::optional<Mesh_vertex_attribute_fetch_info> texcoord_fetch_info;
  std::uint32_t stride;
};

struct Mesh_create_info {
  Mesh_index_format index_format;
  std::uint32_t index_count;
  void const *index_data;
  Mesh_vertex_format vertex_format;
  std::uint32_t vertex_count;
  void const *vertex_data;
};

class Mesh {};
} // namespace graphics
} // namespace marlon

#endif