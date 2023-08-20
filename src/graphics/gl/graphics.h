#ifndef MARLON_GRAPHICS_GL_RENDER_ENGINE_H
#define MARLON_GRAPHICS_GL_RENDER_ENGINE_H

#include <memory>

#include "../graphics.h"
#include "default_render_target.h"
#include "unique_shader_program_handle.h"
#include "unique_texture_handle.h"

namespace marlon {
namespace graphics {
class Gl_graphics : public Graphics {
public:
  Gl_graphics();

  Mesh *create_mesh(Mesh_create_info const &create_info) final;

  void destroy_mesh(Mesh *mesh) noexcept final;

  Texture *create_texture(Texture_create_info const &create_info) final;

  void destroy_texture(Texture *texture) noexcept final;

  Material *create_material(Material_create_info const &create_info) final;

  void destroy_material(Material *material) noexcept final;

  Scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(Scene *scene) noexcept final;

  Surface *create_surface(Surface_create_info const &create_info) final;

  void destroy_surface(Surface *surface) noexcept final;

  Camera *create_camera(Camera_create_info const &create_info) final;

  void destroy_camera(Camera *camera) noexcept final;

  Render_target *get_default_render_target() noexcept;

  void destroy_render_target(Render_target *target) noexcept final;

  void render(Scene *source_scene, Camera *source_camera_instance,
              Render_target *target) final;

private:
  std::unique_ptr<Gl_default_render_target> _default_render_target;
  Gl_unique_shader_program_handle _shader_program;
  Gl_unique_texture_handle _default_base_color_texture;
};
} // namespace graphics
} // namespace marlon

#endif