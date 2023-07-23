#ifndef MARLON_RENDERING_GL_SURFACE_INSTANCE_H
#define MARLON_RENDERING_GL_SURFACE_INSTANCE_H

#include "../surface_instance.h"
#include "scene_node.h"
#include "surface.h"

namespace marlon {
namespace graphics {
class Gl_surface_instance : public Surface_instance {
  friend class Gl_scene;

public:
  class Impl {
  public:
    explicit Impl(Surface_instance_create_info const &create_info) noexcept;
  
    Gl_surface *get_surface() const noexcept {
      return _surface;
    }
    
    Gl_scene_node *get_scene_node() const noexcept {
      return _scene_node;
    }

  private:
    Gl_surface *_surface;
    Gl_scene_node *_scene_node;
  };

  explicit Gl_surface_instance(
      Surface_instance_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif