#ifndef MARLON_RENDERING_RENDER_ENGINE_H
#define MARLON_RENDERING_RENDER_ENGINE_H

#include "../../shared/math/quat.h"
#include "../../shared/math/vec.h"

namespace marlon {
namespace rendering {
class Camera;
struct Camera_create_info;
class Camera_instance;
struct Camera_instance_create_info;
class Material;
struct Material_create_info;
class Mesh;
struct Mesh_create_info;
class Render_destination;
class Render_stream;
struct Render_stream_create_info;
class Scene;
struct Scene_create_info;
class Scene_diff;
struct Scene_diff_create_info;
class Scene_node;
struct Scene_node_create_info;
class Surface;
struct Surface_create_info;
class Surface_instance;
struct Surface_instance_create_info;

class Render_engine {
public:
  virtual ~Render_engine() = default;

  virtual Mesh *create_mesh(Mesh_create_info const &create_info) = 0;

  virtual void destroy_mesh(Mesh *mesh) = 0;

  virtual Material *
  create_material(Material_create_info const &create_info) = 0;

  virtual void destroy_material(Material *material) = 0;

  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) = 0;

  virtual Scene *create_scene(Scene_create_info const &create_info) = 0;

  virtual void destroy_scene(Scene *scene) = 0;

  virtual Scene_diff *
  create_scene_diff(Scene_diff_create_info const &create_info) = 0;

  virtual void destroy_scene_diff(Scene_diff *scene_diff) noexcept = 0;

  virtual void apply_scene_diff(Scene_diff *scene_diff, float factor) = 0;

  virtual void apply_scene_diff(Scene_diff *scene_diff) = 0;

  virtual Scene_node *
  record_scene_node_creation(Scene_diff *scene_diff,
                             Scene_node_create_info const &create_info) = 0;

  virtual void record_scene_node_destruction(Scene_diff *scene_diff,
                                             Scene_node *scene_node) = 0;

  virtual void
  record_scene_node_translation_continuous(Scene_diff *scene_diff,
                                           Scene_node *scene_node,
                                           math::Vec3f const &value) = 0;

  virtual void
  record_scene_node_translation_discontinuous(Scene_diff *scene_diff,
                                              Scene_node *scene_node,
                                              math::Vec3f const &value) = 0;

  virtual void
  record_scene_node_rotation_continuous(Scene_diff *scene_diff,
                                        Scene_node *scene_node,
                                        math::Quatf const &value) = 0;

  virtual void
  record_scene_node_rotation_discontinuous(Scene_diff *scene_diff,
                                           Scene_node *scene_node,
                                           math::Quatf const &value) = 0;

  virtual void record_scene_node_scale_continuous(Scene_diff *scene_diff,
                                                  Scene_node *scene_node,
                                                  float value) = 0;

  virtual void record_scene_node_scale_discontinuous(Scene_diff *scene_diff,
                                                     Scene_node *scene_node,
                                                     float value) = 0;

  virtual Camera *
  record_camera_creation(Scene_diff *scene_diff,
                         Camera_create_info const &create_info) = 0;

  virtual void record_camera_destruction(Scene_diff *scene_diff,
                                         Camera *camera) = 0;

  virtual Camera_instance *record_camera_instance_creation(
      Scene_diff *scene_diff,
      Camera_instance_create_info const &create_info) = 0;

  virtual void
  record_camera_instance_destruction(Scene_diff *scene_diff,
                                     Camera_instance *camera_instance) = 0;

  virtual Surface_instance *record_surface_instance_creation(
      Scene_diff *scene_diff,
      Surface_instance_create_info const &create_info) = 0;

  virtual void
  record_surface_instance_destruction(Scene_diff *scene_diff,
                                      Surface_instance *surface_instance) = 0;

  // Render_destination creation is implementation-specific

  virtual void destroy_render_destination(Render_destination *destination) = 0;

  virtual Render_stream *
  create_render_stream(Render_stream_create_info const &create_info) = 0;

  virtual void destroy_render_stream(Render_stream *render_stream) = 0;

  virtual void render(Render_stream *stream) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};
} // namespace rendering
} // namespace marlon

#endif