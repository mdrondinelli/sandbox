#ifndef MARLON_GRAPHICS_GL_CAMERA_INSTANCE_H
#define MARLON_GRAPHICS_GL_CAMERA_INSTANCE_H

#include "../camera_instance.h"
#include "camera.h"
#include "scene_node.h"

namespace marlon {
namespace graphics {
class Gl_camera_instance : public Camera_instance {
  friend class Gl_graphics;

public:
  class Impl {
  public:
    explicit Impl(Camera_instance_create_info const &create_info) noexcept;

    Gl_camera *get_camera() const noexcept {
      return _camera;
    }

    Gl_scene_node *get_scene_node() const noexcept {
      return _scene_node;
    }

  private:
    Gl_camera *_camera;
    Gl_scene_node *_scene_node;
  };

  explicit Gl_camera_instance(
      Camera_instance_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace graphics
} // namespace marlon

#endif