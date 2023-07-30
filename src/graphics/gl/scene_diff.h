#ifndef MARLON_GRAPHICS_GL_SCENE_DIFF_H
#define MARLON_GRAPHICS_GL_SCENE_DIFF_H

#include <memory>

#include "../scene_diff.h"
#include "scene.h"
#include "scene_node.h"
#include "surface_instance.h"

namespace marlon {
namespace graphics {
class Gl_scene_diff : public Scene_diff {
  friend class Gl_graphics;

public:
  class Impl {
  public:
    explicit Impl(Scene_diff_create_info const &create_info) noexcept;

    Gl_scene_node *
    record_scene_node_creation(Scene_node_create_info const &create_info);

    void record_scene_node_destruction(Gl_scene_node *scene_node);

    void record_scene_node_translation_continuous(Gl_scene_node *scene_node,
                                                  math::Vec3f const &value);

    void record_scene_node_translation_discontinuous(Gl_scene_node *scene_node,
                                                     math::Vec3f const &value);

    void record_scene_node_rotation_continuous(Gl_scene_node *scene_node,
                                               math::Quatf const &value);

    void record_scene_node_rotation_discontinuous(Gl_scene_node *scene_node,
                                                  math::Quatf const &value);

    void record_scene_node_scale_continuous(Gl_scene_node *scene_node,
                                            float value);

    void record_scene_node_scale_discontinuous(Gl_scene_node *scene_node,
                                               float value);

    Gl_camera *record_camera_creation(Camera_create_info const &create_info);

    void record_camera_destruction(Gl_camera *camera);

    Gl_camera_instance *record_camera_instance_creation(
        Camera_instance_create_info const &create_info);

    void
    record_camera_instance_destruction(Gl_camera_instance *camera_instance);

    Gl_surface_instance *record_surface_instance_creation(
        Surface_instance_create_info const &create_info);

    void record_surface_instance_destruction(
        Gl_surface_instance *surface_instance);

    void apply();

    void apply(float factor);

  private:
    void apply_continuous();
    void apply_discontinuous();
    void apply_created_cameras();
    void apply_created_scene_nodes();
    void apply_created_camera_instances();
    void apply_created_surface_instances();
    void apply_destroyed_surface_instances();
    void apply_destroyed_camera_instances();
    void apply_destroyed_scene_nodes();
    void apply_destroyed_cameras();

    Gl_scene *_scene;
    std::vector<std::pair<Gl_scene_node *, math::Vec3f>>
        _continuous_scene_node_translations;
    std::vector<std::pair<Gl_scene_node *, math::Quatf>>
        _continuous_scene_node_rotations;
    std::vector<std::pair<Gl_scene_node *, float>>
        _continuous_scene_node_scales;
    std::vector<std::pair<Gl_scene_node *, math::Vec3f>>
        _discontinuous_scene_node_translations;
    std::vector<std::pair<Gl_scene_node *, math::Quatf>>
        _discontinuous_scene_node_rotations;
    std::vector<std::pair<Gl_scene_node *, float>>
        _discontinuous_scene_node_scales;
    std::vector<std::unique_ptr<Gl_camera>> _created_cameras;
    std::vector<std::unique_ptr<Gl_scene_node>> _created_scene_nodes;
    std::vector<std::unique_ptr<Gl_camera_instance>> _created_camera_instances;
    std::vector<std::unique_ptr<Gl_surface_instance>>
        _created_surface_instances;
    std::vector<Gl_camera_instance *> _destroyed_camera_instances;
    std::vector<Gl_surface_instance *> _destroyed_surface_instances;
    std::vector<Gl_camera *> _destroyed_cameras;
    std::vector<Gl_scene_node *> _destroyed_scene_nodes;
  };

  explicit Gl_scene_diff(Scene_diff_create_info const &create_info) noexcept;

  Gl_scene_node *
  record_scene_node_creation(Scene_node_create_info const &create_info) final;

  void record_scene_node_destruction(Scene_node *scene_node) final;

  void record_scene_node_translation_continuous(Scene_node *scene_node,
                                                math::Vec3f const &value) final;

  void
  record_scene_node_translation_discontinuous(Scene_node *scene_node,
                                              math::Vec3f const &value) final;

  void record_scene_node_rotation_continuous(Scene_node *scene_node,
                                             math::Quatf const &value) final;

  void record_scene_node_rotation_discontinuous(Scene_node *scene_node,
                                                math::Quatf const &value) final;

  void record_scene_node_scale_continuous(Scene_node *scene_node,
                                          float value) final;

  void record_scene_node_scale_discontinuous(Scene_node *scene_node,
                                             float value) final;

  Gl_camera *
  record_camera_creation(Camera_create_info const &create_info) final;

  void record_camera_destruction(Camera *camera) final;

  Gl_camera_instance *record_camera_instance_creation(
      Camera_instance_create_info const &create_info) final;

  void
  record_camera_instance_destruction(Camera_instance *camera_instance) final;

  Gl_surface_instance *record_surface_instance_creation(
      Surface_instance_create_info const &create_info) final;

  void
  record_surface_instance_destruction(Surface_instance *surface_instance) final;

private:
  Impl _impl;
};
} // namespace graphics
} // namespace marlon

#endif