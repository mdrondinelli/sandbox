#ifndef MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H
#define MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H

#include <span>

#include <math/math.h>
#include <util/list.h>

#include "../camera.h"
#include "../scene.h"
#include "./wrappers/wrappers.h"
#include "multiple_buffer.h"
#include "surface_resource.h"
#include "uniform_buffer.h"

namespace marlon::graphics::gl {
struct Cascaded_shadow_map_create_info {
  int texture_resolution;
  int cascade_count;
};

class Cascaded_shadow_map {
public:
  static auto constexpr max_cascade_count = 8;

  struct Intrinsic_state_create_info {};

  class Intrinsic_state {
  public:
    constexpr Intrinsic_state() noexcept = default;

    explicit Intrinsic_state(Intrinsic_state_create_info const &create_info);

    constexpr std::uint32_t shader_program() const noexcept {
      return _shader_program.get();
    }

  private:
    wrappers::Unique_shader_program _shader_program;
  };

  class Cascade {
  public:
    constexpr Cascade() noexcept = default;

    explicit Cascade(std::uint32_t texture, std::int32_t layer);

    constexpr std::uint32_t framebuffer() const noexcept {
      return _framebuffer.get();
    }

    constexpr Uniform_buffer const &uniform_buffer() const noexcept {
      return _uniform_buffers.get();
    }

    void acquire() { _uniform_buffers.acquire(); }

    void release() { _uniform_buffers.release(); }

  private:
    wrappers::Unique_framebuffer _framebuffer;
    Triple_buffer<Uniform_buffer> _uniform_buffers;
    math::Mat3x4f _view_clip_matrix{math::Mat3x4f::identity()};
    float _render_distance{};
    float _pixel_length{};
  };

  Cascaded_shadow_map() = default;

  explicit Cascaded_shadow_map(
      Intrinsic_state const *intrinsic_state,
      Cascaded_shadow_map_create_info const &create_info);

  Cascaded_shadow_map(Cascaded_shadow_map &&other)
      : _intrinsic_state{std::exchange(other._intrinsic_state, nullptr)},
        _texture_resolution{std::exchange(other._texture_resolution, 0)},
        _texture{std::move(other._texture)},
        _uniform_buffers{std::move(other._uniform_buffers)} {
    for (auto &cascade : other._cascades) {
      _cascades.emplace_back(std::move(cascade));
    }
    other._cascades.clear();
  }

  Cascaded_shadow_map &operator=(Cascaded_shadow_map &&other) noexcept {
    _intrinsic_state = std::exchange(other._intrinsic_state, nullptr);
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

  void acquire();

  void release();

  // should only be called when csm is acquired
  void draw(Scene const &scene,
            Camera const &camera,
            math::Vec3f const &light_z_axis,
            Surface_resource const &surface_resource);

private:
  void update_uniform_buffer();

  Intrinsic_state const *_intrinsic_state;
  int _texture_resolution{};
  wrappers::Unique_texture _texture{};
  util::Object_storage<Cascade[max_cascade_count]> _cascade_storage;
  util::List<Cascade> _cascades{_cascade_storage.data(), max_cascade_count};
  Triple_buffer<Uniform_buffer> _uniform_buffers{};
};
} // namespace marlon::graphics::gl

#endif