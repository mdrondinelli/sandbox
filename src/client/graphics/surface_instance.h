#ifndef MARLON_RENDERING_SURFACE_INSTANCE_H
#define MARLON_RENDERING_SURFACE_INSTANCE_H

namespace marlon {
namespace rendering {
class Scene_node;
class Surface;

struct Surface_instance_create_info {
  Surface *surface;
  Scene_node *scene_node;
};

class Surface_instance {};
} // namespace rendering
} // namespace marlon

#endif