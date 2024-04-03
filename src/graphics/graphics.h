#ifndef MARLON_GRAPHICS_GRAPHICS_H
#define MARLON_GRAPHICS_GRAPHICS_H

#include <memory>

#include "../math/quat.h"
#include "../math/vec.h"
#include "render_target.h"
#include "rgb_spectrum.h"
#include "scene.h"
#include "surface_material.h"
#include "surface_mesh.h"
#include "texture.h"

namespace marlon {
namespace graphics {
class Graphics;

struct Render_info {
  Render_target *target;
  Scene *source;
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec2f zoom{1.0f, 1.0f};
  float near_plane_distance{0.01f};
  float far_plane_distance{1000.0f};
};

class Scene_deleter {
public:
  Scene_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Scene *scene) const noexcept;

private:
  Graphics *_owner;
};

class Surface_material_deleter {
public:
  Surface_material_deleter(Graphics *owner = nullptr) noexcept
      : _owner{owner} {}

  void operator()(Surface_material *surface_material) const noexcept;

private:
  Graphics *_owner;
};

class Surface_mesh_deleter {
public:
  Surface_mesh_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Surface_mesh *surface_mesh) const noexcept;

private:
  Graphics *_owner;
};

class Texture_deleter {
public:
  Texture_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Texture *texture) const noexcept;

private:
  Graphics *_owner;
};

using Unique_scene_ptr = std::unique_ptr<Scene, Scene_deleter>;
using Unique_surface_material_ptr =
    std::unique_ptr<Surface_material, Surface_material_deleter>;
using Unique_surface_mesh_ptr =
    std::unique_ptr<Surface_mesh, Surface_mesh_deleter>;
using Unique_texture_ptr = std::unique_ptr<Texture, Texture_deleter>;

class Graphics {
public:
  virtual ~Graphics() = default;

  virtual Surface_material *
  create_surface_material(Surface_material_create_info const &create_info) = 0;

  virtual void
  destroy_surface_material(Surface_material *surface_material) noexcept = 0;

  Unique_surface_material_ptr create_surface_material_unique(
      Surface_material_create_info const &create_info) {
    return Unique_surface_material_ptr{create_surface_material(create_info),
                                       this};
  }

  virtual Surface_mesh *
  create_surface_mesh(Surface_mesh_create_info const &create_info) = 0;

  virtual void destroy_surface_mesh(Surface_mesh *surface_mesh) noexcept = 0;

  Unique_surface_mesh_ptr
  create_surface_mesh_unique(Surface_mesh_create_info const &create_info) {
    return Unique_surface_mesh_ptr{create_surface_mesh(create_info), this};
  }

  virtual Texture *create_texture(Texture_create_info const &create_info) = 0;

  virtual void destroy_texture(Texture *texture) noexcept = 0;

  Unique_texture_ptr
  create_texture_unique(Texture_create_info const &create_info) {
    return Unique_texture_ptr{create_texture(create_info), this};
  }

  virtual Scene *create_scene(Scene_create_info const &create_info) = 0;

  virtual void destroy_scene(Scene *scene) noexcept = 0;

  Unique_scene_ptr create_scene_unique(Scene_create_info const &create_info) {
    return Unique_scene_ptr{create_scene(create_info), this};
  }

  // Render_target creation is implementation-specific

  virtual void destroy_render_target(Render_target *target) noexcept = 0;

  virtual void render(Render_info const &info) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};

inline void Scene_deleter::operator()(Scene *scene) const noexcept {
  if (_owner) {
    _owner->destroy_scene(scene);
  }
}

inline void Surface_material_deleter::operator()(
    Surface_material *surface_material) const noexcept {
  if (_owner) {
    _owner->destroy_surface_material(surface_material);
  }
}

inline void
Surface_mesh_deleter::operator()(Surface_mesh *surface_mesh) const noexcept {
  if (_owner) {
    _owner->destroy_surface_mesh(surface_mesh);
  }
}

inline void Texture_deleter::operator()(Texture *texture) const noexcept {
  if (_owner) {
    _owner->destroy_texture(texture);
  }
}
} // namespace graphics
} // namespace marlon

#endif