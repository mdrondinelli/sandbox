#ifndef MARLON_RENDERING_GL_CAMERA_INSTANCE_H
#define MARLON_RENDERING_GL_CAMERA_INSTANCE_H

#include "../camera_instance.h"
#include "camera.h"
#include "scene_node.h"

namespace marlon {
namespace rendering {
class Gl_camera_instance : public Camera_instance {
public:
  class Impl {
  public:
    explicit Impl(Camera_instance_create_info const &create_info) noexcept;

  private:
    Gl_camera *_camera;
    Gl_scene_node *_scene_node;
  };

  explicit Gl_camera_instance(
      Camera_instance_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif