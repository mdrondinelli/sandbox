#ifndef MARLON_GRAPHICS_SCENE_H
#define MARLON_GRAPHICS_SCENE_H

#include <memory>
#include <optional>

#include <util/set.h>

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
  explicit Scene(Scene_create_info const &create_info) noexcept;

  ~Scene();

  Rgb_spectrum ambient_irradiance() const noexcept {
    return _ambient_irradiance;
  }

  void ambient_irradiance(Rgb_spectrum ambient_irradiance) noexcept {
    _ambient_irradiance = ambient_irradiance;
  }

  std::optional<Directional_light> const &directional_light() const noexcept {
    return _directional_light;
  }

  void directional_light(
      std::optional<Directional_light> const &directional_light) noexcept {
    _directional_light = directional_light;
  }

  util::Set<Surface const *> const &surfaces() const noexcept {
    return _surfaces;
  }

  util::Set<Wireframe const *> const &wireframes() const noexcept {
    return _wireframes;
  }

  void clear() {
    _ambient_irradiance = Rgb_spectrum::black();
    _directional_light = std::nullopt;
    clear_surfaces();
    clear_wireframes();
  }

  void clear_surfaces() noexcept { _surfaces.clear(); }

  void clear_wireframes() noexcept { _wireframes.clear(); }

  void add(Surface const *surface) { _surfaces.emplace(surface); }

  void add(Wireframe const *wireframe) { _wireframes.emplace(wireframe); }

  void remove(Surface const *surface) noexcept { _surfaces.erase(surface); }

  void remove(Wireframe const *wireframe) noexcept {
    _wireframes.erase(wireframe);
  }

private:
  util::Block _memory;
  util::Set<Surface const *> _surfaces;
  util::Set<Wireframe const *> _wireframes;
  Rgb_spectrum _ambient_irradiance{Rgb_spectrum::black()};
  std::optional<Directional_light> _directional_light;
};
} // namespace graphics
} // namespace marlon

#endif