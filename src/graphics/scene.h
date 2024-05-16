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

class Scene {
public:
  virtual Rgb_spectrum get_ambient_irradiance() const noexcept = 0;

  virtual void
  set_ambient_irradiance(Rgb_spectrum ambient_irradiance) noexcept = 0;

  virtual std::optional<Directional_light> const &
  get_directional_light() const noexcept = 0;

  virtual void set_directional_light(
      std::optional<Directional_light> const &directional_light) noexcept = 0;

  virtual void add_surface(Surface const *surface) = 0;

  virtual void remove_surface(Surface const *surface) noexcept = 0;

  virtual void add_wireframe(Wireframe const *wireframe) = 0;

  virtual void remove_wireframe(Wireframe const *wireframe) noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif