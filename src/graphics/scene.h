#ifndef MARLON_GRAPHICS_SCENE_H
#define MARLON_GRAPHICS_SCENE_H

#include <memory>

#include "surface.h"

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

using Unique_surface_ptr = std::unique_ptr<Surface, Surface_deleter>;

class Scene {
public:
  virtual Surface *create_surface(Surface_create_info const &create_info) = 0;

  virtual void destroy_surface(Surface *surface) noexcept = 0;

  Unique_surface_ptr
  create_surface_unique(Surface_create_info const &create_info) {
    return Unique_surface_ptr{create_surface(create_info), this};
  }
};

inline void Surface_deleter::operator()(Surface *surface) const noexcept {
  if (_owner) {
    _owner->destroy_surface(surface);
  }
}
} // namespace graphics
} // namespace marlon

#endif