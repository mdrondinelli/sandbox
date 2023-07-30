#ifndef MARLON_GRAPHICS_SCENE_DIFF_H
#define MARLON_GRAPHICS_SCENE_DIFF_H

#include "../math/quat.h"
#include "../math/vec.h"
#include "camera.h"
#include "camera_instance.h"
#include "scene.h"
#include "scene_node.h"
#include "surface.h"
#include "surface_instance.h"

namespace marlon {
namespace graphics {
struct Scene_diff_create_info {
  Scene *scene{nullptr};
};

class Scene_diff {
public:
  virtual ~Scene_diff() = default;

  virtual Scene_node *
  record_scene_node_creation(Scene_node_create_info const &create_info) = 0;

  virtual void record_scene_node_destruction(Scene_node *scene_node) = 0;

  virtual void
  record_scene_node_translation_continuous(Scene_node *scene_node,
                                           math::Vec3f const &value) = 0;

  virtual void
  record_scene_node_translation_discontinuous(Scene_node *scene_node,
                                              math::Vec3f const &value) = 0;

  virtual void
  record_scene_node_rotation_continuous(Scene_node *scene_node,
                                        math::Quatf const &value) = 0;

  virtual void
  record_scene_node_rotation_discontinuous(Scene_node *scene_node,
                                           math::Quatf const &value) = 0;

  virtual void record_scene_node_scale_continuous(Scene_node *scene_node,
                                                  float value) = 0;

  virtual void record_scene_node_scale_discontinuous(Scene_node *scene_node,
                                                     float value) = 0;

  virtual Camera *
  record_camera_creation(Camera_create_info const &create_info) = 0;

  virtual void record_camera_destruction(Camera *camera) = 0;

  virtual Camera_instance *record_camera_instance_creation(
      Camera_instance_create_info const &create_info) = 0;

  virtual void
  record_camera_instance_destruction(Camera_instance *camera_instance) = 0;

  virtual Surface_instance *record_surface_instance_creation(
      Surface_instance_create_info const &create_info) = 0;

  virtual void
  record_surface_instance_destruction(Surface_instance *surface_instance) = 0;
};
} // namespace graphics
} // namespace marlon

#endif