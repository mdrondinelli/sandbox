#ifndef MARLON_RENDERING_GL_SURFACE_INSTANCE_H
#define MARLON_RENDERING_GL_SURFACE_INSTANCE_H

#include "../surface_instance.h"
#include "scene_node.h"
#include "surface.h"

namespace marlon {
namespace rendering {
class Gl_surface_instance : public Surface_instance {
public:
  explicit Gl_surface_instance(
      Surface_instance_create_info const &create_info) noexcept;

  Gl_surface *surface() const noexcept final;

  Gl_scene_node *node() const noexcept final;

private:
  Gl_surface *_surface;
  Gl_scene_node *_node;
};
} // namespace rendering
} // namespace marlon

#endif