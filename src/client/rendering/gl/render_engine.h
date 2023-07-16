#ifndef MARLON_RENDERING_GL_RENDER_ENGINE_H
#define MARLON_RENDERING_GL_RENDER_ENGINE_H

#include <memory>

#include "../render_engine.h"
#include "camera.h"
#include "camera_instance.h"
#include "default_render_destination.h"
#include "mesh.h"
#include "render_stream.h"
#include "scene.h"
#include "scene_diff.h"
#include "surface.h"
#include "surface_instance.h"
#include "unique_shader_program.h"

namespace marlon {
namespace rendering {
class Gl_render_engine : public Render_engine {
public:
  Gl_render_engine();

  Gl_mesh *create_mesh(Mesh_create_info const &create_info) final;

  void destroy_mesh(Mesh *mesh) final;

  Gl_material *create_material(Material_create_info const &create_info) final;

  void destroy_material(Material *material) final;

  Gl_surface *create_surface(Surface_create_info const &create_info) final;

  void destroy_surface(Surface *surface) final;

  Gl_scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(Scene *scene) final;

  Gl_scene_diff *
  create_scene_diff(Scene_diff_create_info const &create_info) final;

  void destroy_scene_diff(Scene_diff *scene_diff) noexcept final;

  void apply_scene_diff(Scene_diff *scene_diff) final;

  void apply_scene_diff(Scene_diff *scene_diff, float factor) final;

  Gl_scene_node *
  record_scene_node_creation(Scene_diff *scene_diff,
                             Scene_node_create_info const &create_info) final;

  void record_scene_node_destruction(Scene_diff *scene_diff,
                                     Scene_node *scene_node) final;

  void record_scene_node_translation_continuous(Scene_diff *scene_diff,
                                                Scene_node *scene_node,
                                                math::Vec3f const &value) final;

  void
  record_scene_node_translation_discontinuous(Scene_diff *scene_diff,
                                              Scene_node *scene_node,
                                              math::Vec3f const &value) final;

  void record_scene_node_rotation_continuous(Scene_diff *scene_diff,
                                             Scene_node *scene_node,
                                             math::Quatf const &value) final;

  void record_scene_node_rotation_discontinuous(Scene_diff *scene_diff,
                                                Scene_node *scene_node,
                                                math::Quatf const &value) final;

  void record_scene_node_scale_continuous(Scene_diff *scene_diff,
                                          Scene_node *scene_node,
                                          float value) final;

  void record_scene_node_scale_discontinuous(Scene_diff *scene_diff,
                                             Scene_node *scene_node,
                                             float value) final;

  Gl_camera *
  record_camera_creation(Scene_diff *scene_diff,
                         Camera_create_info const &create_info) final;

  void record_camera_destruction(Scene_diff *scene_diff, Camera *camera) final;

  Gl_camera_instance *record_camera_instance_creation(
      Scene_diff *scene_diff,
      Camera_instance_create_info const &create_info) final;

  void
  record_camera_instance_destruction(Scene_diff *scene_diff,
                                     Camera_instance *camera_instance) final;

  Gl_surface_instance *record_surface_instance_creation(
      Scene_diff *scene_diff,
      Surface_instance_create_info const &create_info) final;

  void record_surface_instance_destruction(
      Scene_diff *scene_diff,
      Surface_instance *surface_instance) final;

  Gl_default_render_destination *get_default_render_destination() noexcept;

  void destroy_render_destination(Render_destination *destination) final;

  Gl_render_stream *
  create_render_stream(Render_stream_create_info const &create_info) final;

  void destroy_render_stream(Render_stream *stream) final;

  void render(Render_stream *stream) final;

private:
  std::unique_ptr<Gl_default_render_destination> _default_render_destination;
  Gl_unique_shader_program _shader_program;
};
} // namespace rendering
} // namespace marlon

#endif