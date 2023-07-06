#ifndef MARLON_RENDERING_GL_RENDERER_H
#define MARLON_RENDERING_GL_RENDERER_H

#include <memory>

#include "../renderer.h"
#include "mesh.h"
#include "scene.h"
#include "surface.h"

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
  
  Render_destination *create_default_render_destination() noexcept;
  
  void destroy_render_destination(Render_destination *destination) final;

private:
  std::unique_ptr<Render_destination> _default_render_destination;
};
} // namespace rendering
} // namespace marlon

#endif