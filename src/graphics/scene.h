#ifndef MARLON_GRAPHICS_SCENE_H
#define MARLON_GRAPHICS_SCENE_H

#include <memory>
#include <optional>

#include "directional_light.h"
#include "surface.h"
#include "wireframe.h"

namespace marlon {
namespace graphics {
struct Scene_create_info {
  std::size_t max_surfaces{10000};
  std::size_t max_wireframes{10000};
};

class Scene;

class Surface_deleter {
public:
  Surface_deleter(Scene *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Surface *surface) const noexcept;

private:
  Scene *_owner;
};

class Wireframe_deleter {
public:
  Wireframe_deleter(Scene *owner = nullptr) noexcept : _owner{owner} {}

  void operator()(Wireframe *wireframe) const noexcept;

private:
  Scene *_owner;
};

using Unique_surface_ptr = std::unique_ptr<Surface, Surface_deleter>;
using Unique_wireframe_ptr = std::unique_ptr<Wireframe, Wireframe_deleter>;

class Scene {
public:
  virtual Rgb_spectrum get_ambient_irradiance() const noexcept = 0;

  virtual void
  set_ambient_irradiance(Rgb_spectrum ambient_irradiance) noexcept = 0;

  virtual std::optional<Directional_light> const &
  get_directional_light() const noexcept = 0;

  virtual void set_directional_light(
      std::optional<Directional_light> const &directional_light) noexcept = 0;

  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) noexcept = 0;

  Unique_surface_ptr
  create_surface_unique(Surface_create_info const &create_info) {
    return Unique_surface_ptr{create_surface(create_info), this};
  }

  virtual Wireframe *
  create_wireframe(Wireframe_create_info const &create_info) = 0;

  virtual void destroy_wireframe(Wireframe *wireframe) noexcept = 0;

  Unique_wireframe_ptr
  create_wireframe_unique(Wireframe_create_info const &create_info) {
    return Unique_wireframe_ptr{create_wireframe(create_info), this};
  }
};

inline void Surface_deleter::operator()(Surface *surface) const noexcept {
  if (_owner) {
    _owner->destroy_surface(surface);
  }
}

inline void Wireframe_deleter::operator()(Wireframe *wireframe) const noexcept {
  if (_owner) {
    _owner->destroy_wireframe(wireframe);
  }
}
} // namespace graphics
} // namespace marlon

#endif