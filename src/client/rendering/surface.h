#ifndef MARLONR_RENDERING_SURFACE_H
#define MARLONR_RENDERING_SURFACE_H

namespace marlon {
namespace rendering {
class Material;
class Mesh;

struct Surface_create_info {
  Mesh *mesh;
  Material *material;
};

class Surface {
public:
  virtual ~Surface() = default;

  virtual Mesh *mesh() const noexcept = 0;

  virtual Material *material() const noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif