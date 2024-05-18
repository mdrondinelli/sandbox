#ifndef MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H
#define MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H

#include <span>

#include <math/math.h>
#include <util/list.h>

#include "./wrappers/wrappers.h"
#include "multiple_buffer.h"
#include "uniform_buffer.h"

namespace marlon::graphics::gl {
struct Cascaded_shadow_map_create_info {
  int texture_resolution;
  int cascade_count;
};

class Cascaded_shadow_map {
public:
  static auto constexpr max_cascade_count = 8;

  class Cascade {
  public:
    constexpr Cascade() noexcept = default;

    explicit Cascade(std::uint32_t texture, std::int32_t layer);

    constexpr std::uint32_t framebuffer() const noexcept {
      return _framebuffer.get();
    }

    math::Mat3x4f const &view_clip_matrix() const noexcept {
      return _view_clip_matrix;
    }

    void view_clip_matrix(math::Mat3x4f const &m) noexcept {
      _view_clip_matrix = m;
    }

    float render_distance() const noexcept { return _render_distance; }

    void render_distance(float d) noexcept { _render_distance = d; }

    float pixel_length() const noexcept { return _pixel_length; }

    void pixel_length(float l) { _pixel_length = l; }

  private:
    wrappers::Unique_framebuffer _framebuffer;
    math::Mat3x4f _view_clip_matrix{math::Mat3x4f::identity()};
    float _render_distance{};
    float _pixel_length{};
  };

  Cascaded_shadow_map() = default;

  explicit Cascaded_shadow_map(
      Cascaded_shadow_map_create_info const &create_info);

  Cascaded_shadow_map(Cascaded_shadow_map &&other)
      : _texture_resolution{std::exchange(other._texture_resolution, 0)},
        _texture{std::move(other._texture)},
        _uniform_buffers{std::move(other._uniform_buffers)} {
    for (auto &cascade : other._cascades) {
      _cascades.emplace_back(std::move(cascade));
    }
    other._cascades.clear();
  }

  Cascaded_shadow_map &operator=(Cascaded_shadow_map &&other) noexcept {
    _texture_resolution = std::exchange(other._texture_resolution, 0);
    _texture = std::move(other._texture);
    _cascades.clear();
    for (auto &cascade : other._cascades) {
      _cascades.emplace_back(std::move(cascade));
    }
    other._cascades.clear();
    _uniform_buffers = std::move(other._uniform_buffers);
    return *this;
  }

  operator bool() const noexcept { return cascade_count() != 0; }

  int texture_resolution() const noexcept { return _texture_resolution; }

  std::uint32_t texture() const noexcept { return _texture.get(); }

  int cascade_count() const noexcept { return _cascades.size(); }

  std::span<Cascade const> cascades() const noexcept {
    return {_cascades.data(), _cascades.data() + _cascades.size()};
  }

  std::span<Cascade> cascades() noexcept {
    return {_cascades.data(), _cascades.data() + _cascades.size()};
  }

  std::uint32_t uniform_buffer() const noexcept {
    return _uniform_buffers.get().get();
  }

  void update_frusta(math::Vec3f const &camera_position,
                     math::Vec3f const &camera_direction,
                     math::Vec2f const &camera_zoom,
                     float camera_near_plane_distance,
                     float camera_csm_render_distance,
                     int camera_csm_cascade_count,
                     math::Vec3f const &light_z_axis);

  void acquire_uniform_buffer();

  void update_uniform_buffer();

  void release_uniform_buffer();

private:
  int _texture_resolution{};
  wrappers::Unique_texture _texture{};
  util::Object_storage<Cascade[max_cascade_count]> _cascade_storage;
  util::List<Cascade> _cascades{_cascade_storage.data(), max_cascade_count};
  Triple_buffer<Uniform_buffer> _uniform_buffers{};
};
} // namespace marlon::graphics::gl

#endif