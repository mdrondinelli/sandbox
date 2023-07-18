#ifndef MARLONR_RENDERING_SURFACE_H
#define MARLONR_RENDERING_SURFACE_H

namespace marlon {
namespace rendering {
class Material;
class Mesh;

struct Surface_create_info {
  Material *material;
  Mesh *mesh;
};

class Surface {};
} // namespace rendering
} // namespace marlon

#endif