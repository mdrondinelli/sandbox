#ifndef MARLON_RENDERING_SCENE_H
#define MARLON_RENDERING_SCENE_H

namespace marlon {
namespace rendering {
class Camera_instance;
struct Camera_instance_create_info;
class Scene_node;
struct Scene_node_create_info;
class Surface_instance;
struct Surface_instance_create_info;

struct Scene_create_info {};

class Scene {
public:
  virtual ~Scene() = default;

  virtual Scene_node *
  create_scene_node(Scene_node_create_info const &create_info) = 0;

  virtual void destroy_scene_node(Scene_node *node) = 0;

  virtual Camera_instance *
  create_camera_instance(Camera_instance_create_info const &create_info) = 0;

  virtual void destroy_camera_instance(Camera_instance *instance) = 0;

  virtual Surface_instance *
  create_surface_instance(Surface_instance_create_info const &create_info) = 0;

  virtual void destroy_surface_instance(Surface_instance *distance) = 0;
};
} // namespace rendering
} // namespace marlon

#endif