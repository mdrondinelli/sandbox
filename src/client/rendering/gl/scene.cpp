#include "scene.h"

#include "camera_instance.h"
#include "scene_node.h"

namespace marlon {
namespace rendering {
Gl_scene_node *
Gl_scene::create_scene_node(Scene_node_create_info const &create_info) {
  return new Gl_scene_node(create_info);
}

void Gl_scene::destroy_scene_node(Scene_node *node) { delete node; }

Gl_camera_instance *Gl_scene::create_camera_instance(
    Camera_instance_create_info const &create_info) {
  return new Gl_camera_instance{create_info};
}

void Gl_scene::destroy_camera_instance(Camera_instance *instance) {
  delete instance;
}

Gl_surface_instance *Gl_scene::create_surface_instance(
    Surface_instance_create_info const &create_info) {
  return new Gl_surface_instance{create_info};
}

void Gl_scene::destroy_surface_instance(Surface_instance *instance) {
  delete instance;
}
} // namespace rendering
} // namespace marlon