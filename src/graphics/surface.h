#ifndef MARLONR_GRAPHICS_SURFACE_H
#define MARLONR_GRAPHICS_SURFACE_H

#include "material.h"
#include "mesh.h"

namespace marlon {
namespace graphics {
// class Mesh;

// struct Surface_create_info {
//   Material material;
//   Mesh *mesh;
// };

class Surface {
public:
  Material material;
  Mesh *mesh;
};
} // namespace graphics
} // namespace marlon

#endif