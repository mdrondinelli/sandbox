#ifndef MARLON_RENDERING_GL_CAMERA_INSTANCE_H
#define MARLON_RENDERING_GL_CAMERA_INSTANCE_H

#include "../camera_instance.h"
#include "camera.h"
#include "scene_node.h"

namespace marlon {
namespace rendering {
class Gl_camera_instance : public Camera_instance {
public:
  explicit Gl_camera_instance(
      Camera_instance_create_info const &create_info) noexcept;

  Gl_camera *camera() const noexcept override;

  Gl_scene_node *node() const noexcept override;

private:
  Gl_camera *_camera;
  Gl_scene_node *_node;
};
} // namespace rendering
} // namespace marlon

#endif