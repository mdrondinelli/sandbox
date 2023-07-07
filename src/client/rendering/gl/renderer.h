#ifndef MARLON_RENDERING_GL_RENDERER_H
#define MARLON_RENDERING_GL_RENDERER_H

#include <memory>

#include "../renderer.h"
#include "default_render_destination.h"
#include "mesh.h"
#include "render.h"
#include "scene.h"
#include "surface.h"
#include "unique_shader_program.h"

namespace marlon {
namespace rendering {
class Gl_renderer : public Renderer {
public:
  Gl_renderer();

  Gl_camera *create_camera(Camera_create_info const &create_info) final;

  void destroy_camera(Camera *camera) final;

  Gl_mesh *create_mesh(Mesh_create_info const &create_info) final;

  void destroy_mesh(Mesh *mesh) final;

  Gl_material *create_material(Material_create_info const &create_info) final;

  void destroy_material(Material *material) final;

  Gl_surface *create_surface(Surface_create_info const &create_info) final;

  void destroy_surface(Surface *surface) final;

  Gl_scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(Scene *scene) final;

  Gl_default_render_destination *create_default_render_destination() noexcept;

  void destroy_render_destination(Render_destination *destination) final;

  Gl_render *create_render(Render_create_info const &create_info) final;

  void destroy_render(Render *render) final;

  void render(Render *render) final;

private:
  std::unique_ptr<Gl_default_render_destination> _default_render_destination;
  Gl_unique_shader_program _shader_program;
};
} // namespace rendering
} // namespace marlon

#endif