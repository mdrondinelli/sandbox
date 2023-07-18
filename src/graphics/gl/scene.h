#ifndef MARLON_RENDERING_GL_SCENE_H
#define MARLON_RENDERING_GL_SCENE_H

#include <memory>
#include <unordered_set>

#include "../scene.h"
#include "camera.h"
#include "camera_instance.h"
#include "scene_node.h"
#include "surface_instance.h"

namespace marlon {
namespace graphics {
class Gl_scene : public Scene {
  friend class Gl_graphics;
  friend class Gl_scene_diff;

public:
  class Impl {
  public:
    explicit Impl(Scene_create_info const &create_info) noexcept;

    ~Impl();

    void acquire_scene_node(std::unique_ptr<Gl_scene_node> scene_node);

    bool release_scene_node(Gl_scene_node *scene_node);

    void acquire_camera(std::unique_ptr<Gl_camera> camera);

    bool release_camera(Gl_camera *camera);

    void acquire_camera_instance(
        std::unique_ptr<Gl_camera_instance> camera_instance);

    bool release_camera_instance(Gl_camera_instance *camera_instance);

    void acquire_surface_instance(
        std::unique_ptr<Gl_surface_instance> surface_instance);

    bool release_surface_instance(Gl_surface_instance *surface_instance);

    void draw_surface_instances(std::uint32_t shader_program,
                                std::int32_t model_view_matrix_location,
                                std::int32_t model_view_clip_matrix_location,
                                std::int32_t albedo_location,
                                math::Mat4x4f const &view_matrix,
                                math::Mat4x4f const &view_clip_matrix);

  private:
    std::unordered_set<Gl_scene_node *> _scene_nodes;
    std::unordered_set<Gl_camera *> _cameras;
    std::unordered_set<Gl_camera_instance *> _camera_instances;
    std::unordered_set<Gl_surface_instance *> _surface_instances;
  };

  explicit Gl_scene(Scene_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif