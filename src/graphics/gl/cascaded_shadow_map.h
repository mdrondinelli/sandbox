#ifndef MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H
#define MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H

#include <span>

#include <math/math.h>
#include <util/list.h>

#include "wrappers/wrappers.h"

namespace marlon::graphics::gl {
struct Cascaded_shadow_map_create_info {
  int cascade_count;
  int cascade_resolution;
};

class Cascaded_shadow_map {
public:
  static auto constexpr max_cascade_count = 8;

  class Cascade {
  public:
    constexpr Cascade() noexcept = default;

    explicit Cascade(int resolution);

    constexpr std::uint32_t texture() const noexcept { return _texture.get(); }

    constexpr std::uint32_t framebuffer() const noexcept {
      return _framebuffer.get();
    }

    math::Mat3x4f const &view_clip_matrix() const noexcept {
      return _view_clip_matrix;
    }

    void view_clip_matrix(math::Mat3x4f const &m) noexcept {
      _view_clip_matrix = m;
    }

  private:
    wrappers::Unique_texture _texture;
    wrappers::Unique_framebuffer _framebuffer;
    math::Mat3x4f _view_clip_matrix{math::Mat3x4f::identity()};
  };

  Cascaded_shadow_map() = default;

  explicit Cascaded_shadow_map(
      Cascaded_shadow_map_create_info const &create_info);

  Cascaded_shadow_map(Cascaded_shadow_map &&other) {
    for (auto &cascade : other._cascades) {
      _cascades.emplace_back(std::move(cascade));
    }
    other._cascades.clear();
    _cascade_resolution = std::exchange(other._cascade_resolution, 0);
  }

  Cascaded_shadow_map &operator=(Cascaded_shadow_map &&other) noexcept {
    _cascades.clear();
    for (auto &cascade : other._cascades) {
      _cascades.emplace_back(std::move(cascade));
    }
    other._cascades.clear();
    _cascade_resolution = std::exchange(other._cascade_resolution, 0);
    return *this;
  }

  int cascade_count() const noexcept { return _cascades.size(); }

  int cascade_resolution() const noexcept { return _cascade_resolution; }

  std::span<Cascade const> cascades() const noexcept {
    return {_cascades.data(), _cascades.data() + _cascades.size()};
  }

  std::span<Cascade> cascades() noexcept {
    return {_cascades.data(), _cascades.data() + _cascades.size()};
  }

private:
  util::Object_storage<Cascade[max_cascade_count]> _cascade_storage;
  util::List<Cascade> _cascades{_cascade_storage.data(), max_cascade_count};
  int _cascade_resolution{};
};
} // namespace marlon::graphics::gl

#endif