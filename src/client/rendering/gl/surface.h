#ifndef MARLON_RENDERING_GL_SURFACE_H
#define MARLON_RENDERING_GL_SURFACE_H

#include "../surface.h"
#include "material.h"
#include "mesh.h"

namespace marlon {
namespace rendering {
class Gl_surface : public Surface {
  Gl_mesh *_mesh;
  Gl_material *_material;

public:
  explicit Gl_surface(Surface_create_info const &create_info) noexcept;

  Gl_mesh *mesh() const noexcept final;

  Gl_material *material() const noexcept final;
};
} // namespace rendering
} // namespace marlon

#endif