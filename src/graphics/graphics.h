#ifndef MARLON_GRAPHICS_GRAPHICS_H
#define MARLON_GRAPHICS_GRAPHICS_H

#include <memory>

#include "../math/quat.h"
#include "../math/vec.h"
#include "material.h"
#include "mesh.h"
#include "render_target.h"
#include "rgb_spectrum.h"
#include "scene.h"
#include "surface.h"
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

class Mesh_deleter {
public:
  Mesh_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Mesh *mesh) const noexcept;

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

class Material_deleter {
public:
  Material_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Material *material) const noexcept;

private:
  Graphics *_owner;
};

class Scene_deleter {
public:
  Scene_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Scene *scene) const noexcept;

private:
  Graphics *_owner;
};

class Surface_deleter {
public:
  Surface_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Surface *surface) const noexcept;

private:
  Graphics *_owner;
};

using Unique_mesh_ptr = std::unique_ptr<Mesh, Mesh_deleter>;
using Unique_texture_ptr = std::unique_ptr<Texture, Texture_deleter>;
using Unique_material_ptr = std::unique_ptr<Material, Material_deleter>;
using Unique_scene_ptr = std::unique_ptr<Scene, Scene_deleter>;
using Unique_surface_ptr = std::unique_ptr<Surface, Surface_deleter>;

class Graphics {
public:
  virtual ~Graphics() = default;

  virtual Mesh *create_mesh(Mesh_create_info const &create_info) = 0;

  virtual void destroy_mesh(Mesh *mesh) noexcept = 0;

  Unique_mesh_ptr create_mesh_unique(Mesh_create_info const &create_info) {
    return Unique_mesh_ptr{create_mesh(create_info), this};
  }

  virtual Texture *create_texture(Texture_create_info const &create_info) = 0;

  virtual void destroy_texture(Texture *texture) noexcept = 0;

  Unique_texture_ptr
  create_texture_unique(Texture_create_info const &create_info) {
    return Unique_texture_ptr{create_texture(create_info), this};
  }

  virtual Material *
  create_material(Material_create_info const &create_info) = 0;

  virtual void destroy_material(Material *material) noexcept = 0;

  Unique_material_ptr
  create_material_unique(Material_create_info const &create_info) {
    return Unique_material_ptr{create_material(create_info), this};
  }

  virtual Scene *create_scene(Scene_create_info const &create_info) = 0;

  virtual void destroy_scene(Scene *scene) noexcept = 0;

  Unique_scene_ptr create_scene_unique(Scene_create_info const &create_info) {
    return Unique_scene_ptr{create_scene(create_info), this};
  }

  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) noexcept = 0;

  Unique_surface_ptr
  create_surface_unique(Surface_create_info const &create_info) {
    return Unique_surface_ptr{create_surface(create_info), this};
  }

  // Render_target creation is implementation-specific

  virtual void destroy_render_target(Render_target *target) noexcept = 0;

  virtual void render(Render_info const &info) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};

inline void Mesh_deleter::operator()(Mesh *mesh) const noexcept {
  if (_owner) {
    _owner->destroy_mesh(mesh);
  }
}

inline void Texture_deleter::operator()(Texture *texture) const noexcept {
  if (_owner) {
    _owner->destroy_texture(texture);
  }
}

inline void Material_deleter::operator()(Material *material) const noexcept {
  if (_owner) {
    _owner->destroy_material(material);
  }
}

inline void Surface_deleter::operator()(Surface *surface) const noexcept {
  if (_owner) {
    _owner->destroy_surface(surface);
  }
}

inline void Scene_deleter::operator()(Scene *scene) const noexcept {
  if (_owner) {
    _owner->destroy_scene(scene);
  }
}
} // namespace graphics
} // namespace marlon

#endif