#ifndef MARLON_GRAPHICS_SCENE_H
#define MARLON_GRAPHICS_SCENE_H

#include <optional>

#include <util/set.h>

#include "directional_light.h"
#include "surface.h"

namespace marlon {
namespace graphics {
struct Scene_create_info {
  util::Size max_surfaces{10000};
};

class Scene {
public:
  Scene() = default;

  explicit Scene(Scene_create_info const &create_info) noexcept;

  ~Scene();

  Scene(Scene &&other) noexcept;

  Scene &operator=(Scene &&other) noexcept;

  util::Set<Surface const *> const &surfaces() const noexcept {
    return _surfaces;
  }

  void add(Surface const *surface) {
    _surfaces.emplace(surface);
  }

  void remove(Surface const *surface) noexcept {
    _surfaces.erase(surface);
  }

  void clear_surfaces() noexcept {
    _surfaces.clear();
  }

  std::optional<Directional_light> const &sun() const noexcept {
    return _sun;
  }

  void sun(std::optional<Directional_light> const &sun) noexcept {
    _sun = sun;
  }

  Rgb_spectrum sky_irradiance() const noexcept {
    return _sky_irradiance;
  }

  void sky_irradiance(Rgb_spectrum sky_irradiance) noexcept {
    _sky_irradiance = sky_irradiance;
  }

  Rgb_spectrum ground_albedo() const noexcept {
    return _ground_albedo;
  }

  void ground_albedo(Rgb_spectrum ground_albedo) {
    _ground_albedo = ground_albedo;
  }

private:
  void swap(Scene &other) noexcept;

  util::Block _memory;
  util::Set<Surface const *> _surfaces;
  std::optional<Directional_light> _sun;
  Rgb_spectrum _sky_irradiance{Rgb_spectrum::black()};
  Rgb_spectrum _ground_albedo{Rgb_spectrum{0.25f}};
};
} // namespace graphics
} // namespace marlon

#endif
