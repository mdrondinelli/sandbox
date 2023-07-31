#ifndef MARLON_GRAPHICS_GL_RENDER_ENGINE_H
#define MARLON_GRAPHICS_GL_RENDER_ENGINE_H

#include <memory>

#include "../graphics.h"
#include "camera.h"
#include "camera_instance.h"
#include "default_render_target.h"
#include "mesh.h"
#include "scene.h"
#include "scene_diff.h"
#include "surface.h"
#include "surface_instance.h"
#include "texture.h"
#include "unique_shader_program_handle.h"

namespace marlon {
namespace graphics {
class Gl_graphics : public Graphics {
public:
  Gl_graphics();

  Gl_texture *create_texture(Texture_create_info const &create_info) final;

  void destroy_texture(Texture *texture) noexcept final;

  Gl_mesh *create_mesh(Mesh_create_info const &create_info) final;

  void destroy_mesh(Mesh *mesh) noexcept final;

  Gl_material *create_material(Material_create_info const &create_info) final;

  void destroy_material(Material *material) noexcept final;

  Gl_surface *create_surface(Surface_create_info const &create_info) final;

  void destroy_surface(Surface *surface) noexcept final;

  Gl_scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(Scene *scene) noexcept final;

  Gl_scene_diff *
  create_scene_diff(Scene_diff_create_info const &create_info) final;

  void destroy_scene_diff(Scene_diff *scene_diff) noexcept final;

  void apply_scene_diff(Scene_diff *scene_diff) final;

  void apply_scene_diff(Scene_diff *scene_diff, float factor) final;

  Gl_default_render_target *get_default_render_target() noexcept;

  void destroy_render_target(Render_target *target) noexcept final;

  void render(Scene *source_scene, Camera_instance *source_camera_instance,
              Render_target *target) final;

private:
  std::unique_ptr<Gl_default_render_target> _default_render_target;
  Gl_unique_shader_program_handle _shader_program;
  Gl_unique_texture_handle _default_base_color_texture;
};
} // namespace graphics
} // namespace marlon

#endif