#ifndef MARLON_RENDERING_GL_SCENE_H
#define MARLON_RENDERING_GL_SCENE_H

#include "../scene.h"
#include "scene_node.h"
#include "camera_instance.h"

namespace marlon {
namespace rendering {
class Gl_scene : public Scene {
public:
  Gl_scene_node *
  create_scene_node(Scene_node_create_info const &create_info) override;

  void destroy_scene_node(Scene_node *node) override;

  Gl_camera_instance *create_camera_instance(
      Camera_instance_create_info const &create_info) override;

  void destroy_camera_instance(Camera_instance *instance) override;

  Surface_instance *create_surface_instance(
      Surface_instance_create_info const &create_info) override;

  void destroy_surface_instance(Surface_instance *instance) override;
};
} // namespace rendering
} // namespace marlon

#endif