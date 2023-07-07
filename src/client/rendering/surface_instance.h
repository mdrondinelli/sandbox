#ifndef MARLON_RENDERING_SURFACE_INSTANCE_H
#define MARLON_RENDERING_SURFACE_INSTANCE_H

namespace marlon {
namespace rendering {
class Scene_node;
class Surface;

struct Surface_instance_create_info {
  Surface *surface;
  Scene_node *node;
};

class Surface_instance {
public:
  virtual ~Surface_instance() = default;

  virtual Surface *surface() const noexcept = 0;

  virtual Scene_node *node() const noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif