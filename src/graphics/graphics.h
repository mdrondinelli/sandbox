#ifndef MARLON_GRAPHICS_GRAPHICS_H
#define MARLON_GRAPHICS_GRAPHICS_H

#include <memory>

#include "../math/quat.h"
#include "../math/vec.h"
#include "camera.h"
#include "render_stream.h"
#include "render_target.h"
#include "rgb_spectrum.h"
#include "scene.h"
#include "surface_material.h"
#include "surface_mesh.h"
#include "texture.h"
#include "wireframe_mesh.h"

namespace marlon {
namespace graphics {
class Graphics;

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

class Wireframe_mesh_deleter {
public:
  Wireframe_mesh_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Wireframe_mesh *wireframe_mesh) const noexcept;

private:
  Graphics *_owner;
};

class Render_target_deleter {
public:
  Render_target_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Render_target *render_target) const noexcept;

private:
  Graphics *_owner;
};

class Render_stream_deleter {
public:
  Render_stream_deleter(Graphics *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Render_stream *render_stream) const noexcept;

private:
  Graphics *_owner;
};

using Unique_texture = std::unique_ptr<Texture, Texture_deleter>;
using Unique_surface_mesh = std::unique_ptr<Surface_mesh, Surface_mesh_deleter>;
using Unique_wireframe_mesh =
    std::unique_ptr<Wireframe_mesh, Wireframe_mesh_deleter>;
using Unique_render_target =
    std::unique_ptr<Render_target, Render_target_deleter>;
using Unique_render_stream =
    std::unique_ptr<Render_stream, Render_stream_deleter>;

struct Graphics_implementation_info {
  int cascaded_shadow_map_max_cascade_count;
};

class Graphics {
public:
  virtual ~Graphics() = default;

  virtual Graphics_implementation_info const &
  implementation_info() const noexcept = 0;

  Unique_texture create_texture_unique(Texture_create_info const &create_info) {
    return Unique_texture{create_texture(create_info), this};
  }

  virtual Texture *create_texture(Texture_create_info const &create_info) = 0;

  virtual void destroy_texture(Texture *texture) noexcept = 0;

  Unique_surface_mesh
  create_surface_mesh_unique(Surface_mesh_create_info const &create_info) {
    return Unique_surface_mesh{create_surface_mesh(create_info), this};
  }

  virtual Surface_mesh *
  create_surface_mesh(Surface_mesh_create_info const &create_info) = 0;

  virtual void destroy_surface_mesh(Surface_mesh *surface_mesh) noexcept = 0;

  Unique_wireframe_mesh
  create_wireframe_mesh_unique(Wireframe_mesh_create_info const &create_info) {
    return Unique_wireframe_mesh{create_wireframe_mesh(create_info), this};
  }

  virtual Wireframe_mesh *
  create_wireframe_mesh(Wireframe_mesh_create_info const &create_info) = 0;

  virtual void
  destroy_wireframe_mesh(Wireframe_mesh *wireframe_mesh) noexcept = 0;

  // Render_target creation is implementation-specific

  virtual void destroy_render_target(Render_target *target) noexcept = 0;

  Unique_render_stream
  create_render_stream_unique(Render_stream_create_info const &create_info) {
    return Unique_render_stream{create_render_stream(create_info), this};
  }

  virtual Render_stream *
  create_render_stream(Render_stream_create_info const &create_info) = 0;

  virtual void destroy_render_stream(Render_stream *render_stream) = 0;

  // TODO: interface for compositing or figure out how multi view rendering will
  // work compositing could be used for first person view models
};

inline void Texture_deleter::operator()(Texture *texture) const noexcept {
  if (_owner) {
    _owner->destroy_texture(texture);
  }
}

inline void
Surface_mesh_deleter::operator()(Surface_mesh *surface_mesh) const noexcept {
  if (_owner) {
    _owner->destroy_surface_mesh(surface_mesh);
  }
}

inline void Wireframe_mesh_deleter::operator()(
    Wireframe_mesh *wireframe_mesh) const noexcept {
  if (_owner) {
    _owner->destroy_wireframe_mesh(wireframe_mesh);
  }
}

inline void
Render_target_deleter::operator()(Render_target *render_target) const noexcept {
  if (_owner) {
    _owner->destroy_render_target(render_target);
  }
}

inline void
Render_stream_deleter::operator()(Render_stream *render_stream) const noexcept {
  if (_owner) {
    _owner->destroy_render_stream(render_stream);
  }
}
} // namespace graphics
} // namespace marlon

#endif