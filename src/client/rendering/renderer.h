#ifndef MARLON_RENDERING_RENDERER_H
#define MARLON_RENDERING_RENDERER_H

namespace marlon {
namespace rendering {
class Camera;
struct Camera_create_info;
class Material;
struct Material_create_info;
class Mesh;
struct Mesh_create_info;
class Render_destination;
class Render_group;
struct Render_group_create_info;
class Scene;
struct Scene_create_info;
class Surface;
struct Surface_create_info;

class Renderer {
public:
  virtual ~Renderer() = default;

  virtual Camera *create_camera(Camera_create_info const &create_info) = 0;

  virtual void destroy_camera(Camera *camera) = 0;

  virtual Mesh *create_mesh(Mesh_create_info const &create_info) = 0;

  virtual void destroy_mesh(Mesh *mesh) = 0;

  virtual Material *
  create_material(Material_create_info const &create_info) = 0;

  virtual void destroy_material(Material *material) = 0;

  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) = 0;

  virtual Scene *create_scene(Scene_create_info const &create_info) = 0;

  virtual void destroy_scene(Scene *scene) = 0;

  virtual void destroy_render_destination(Render_destination *destination) = 0;

  virtual Render_group *
  create_render_group(Render_group_create_info const &create_info) = 0;

  virtual void destroy_render_group(Render_group *group) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};
} // namespace rendering
} // namespace marlon

#endif