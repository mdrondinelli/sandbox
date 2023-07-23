#ifndef MARLONR_GRAPHICS_SURFACE_H
#define MARLONR_GRAPHICS_SURFACE_H

namespace marlon {
namespace graphics {
class Material;
class Mesh;

struct Surface_create_info {
  Material *material;
  Mesh *mesh;
};

class Surface {};
} // namespace graphics
} // namespace marlon

#endif