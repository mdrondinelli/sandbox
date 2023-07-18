#ifndef MARLON_RENDERING_RENDER_ENGINE_H
#define MARLON_RENDERING_RENDER_ENGINE_H

#include <memory>

#include "../math/quat.h"
#include "../math/vec.h"

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
class Render_target;
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

class Graphics;

class Material_deleter {
public:
  Material_deleter(Graphics *owner) noexcept : _owner{owner} {}

  void operator()(Material *material) const noexcept;

private:
  Graphics *_owner;
};

class Mesh_deleter {
public:
  Mesh_deleter(Graphics *owner) noexcept : _owner{owner} {}

  void operator()(Mesh *mesh) const noexcept;

private:
  Graphics *_owner;
};

class Surface_deleter {
public:
  Surface_deleter(Graphics *owner) noexcept : _owner{owner} {}

  void operator()(Surface *surface) const noexcept;

private:
  Graphics *_owner;
};

class Scene_deleter {
public:
  Scene_deleter(Graphics *owner) noexcept : _owner{owner} {}

  void operator()(Scene *scene) const noexcept;

private:
  Graphics *_owner;
};

class Scene_diff_deleter {
public:
  Scene_diff_deleter(Graphics *owner) noexcept : _owner{owner} {}

  void operator()(Scene_diff *scene_diff) const noexcept;

private:
  Graphics *_owner;
};

using Unique_material_ptr = std::unique_ptr<Material, Material_deleter>;
using Unique_mesh_ptr = std::unique_ptr<Mesh, Mesh_deleter>;
using Unique_surface_ptr = std::unique_ptr<Surface, Surface_deleter>;
using Unique_scene_ptr = std::unique_ptr<Scene, Scene_deleter>;
using Unique_scene_diff_ptr = std::unique_ptr<Scene_diff, Scene_diff_deleter>;

class Graphics {
public:
  virtual ~Graphics() = default;

  virtual Material *
  create_material(Material_create_info const &create_info) = 0;

  virtual void destroy_material(Material *material) noexcept = 0;

  Unique_material_ptr
  create_material_unique(Material_create_info const &create_info) {
    return Unique_material_ptr{create_material(create_info), this};
  }

  virtual Mesh *create_mesh(Mesh_create_info const &create_info) = 0;

  virtual void destroy_mesh(Mesh *mesh) noexcept = 0;

  Unique_mesh_ptr create_mesh_unique(Mesh_create_info const &create_info) {
    return Unique_mesh_ptr{create_mesh(create_info), this};
  }

  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) noexcept = 0;

  Unique_surface_ptr
  create_surface_unique(Surface_create_info const &create_info) {
    return Unique_surface_ptr{create_surface(create_info), this};
  }

  virtual Scene *create_scene(Scene_create_info const &create_info) = 0;

  virtual void destroy_scene(Scene *scene) noexcept = 0;

  Unique_scene_ptr create_scene_unique(Scene_create_info const &create_info) {
    return Unique_scene_ptr{create_scene(create_info), this};
  }

  virtual Scene_diff *
  create_scene_diff(Scene_diff_create_info const &create_info) = 0;

  virtual void destroy_scene_diff(Scene_diff *scene_diff) noexcept = 0;

  Unique_scene_diff_ptr
  create_scene_diff_unique(Scene_diff_create_info const &create_info) {
    return Unique_scene_diff_ptr{create_scene_diff(create_info), this};
  }

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

  // Render_target creation is implementation-specific

  virtual void destroy_render_target(Render_target *target) noexcept = 0;

  virtual void render(Scene *source_scene,
                      Camera_instance *source_camera_instance,
                      Render_target *target) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};

inline void Material_deleter::operator()(Material *material) const noexcept {
  _owner->destroy_material(material);
}

inline void Mesh_deleter::operator()(Mesh *mesh) const noexcept {
  _owner->destroy_mesh(mesh);
}

inline void Surface_deleter::operator()(Surface *surface) const noexcept {
  _owner->destroy_surface(surface);
}

inline void Scene_deleter::operator()(Scene *scene) const noexcept {
  _owner->destroy_scene(scene);
}

inline void
Scene_diff_deleter::operator()(Scene_diff *scene_diff) const noexcept {
  _owner->destroy_scene_diff(scene_diff);
}
} // namespace rendering
} // namespace marlon

#endif