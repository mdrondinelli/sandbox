#ifndef MARLON_GL_RENDER_STREAM_H
#define MARLON_GL_RENDER_STREAM_H

#include <chrono>

#include "../render_stream.h"
#include "./wrappers/unique_shader_program.h"
#include "cascaded_shadow_map.h"
#include "render_target.h"
#include "surface_resource.h"
#include "temporal_antialiasing_resource.h"
#include "visibility_buffer.h"

namespace marlon {
namespace graphics {
namespace gl {
class Render_stream final : public graphics::Render_stream {
public:
  struct Intrinsic_state_create_info {};

  class Intrinsic_state {
  public:
    constexpr Intrinsic_state() noexcept = default;

    Intrinsic_state(Intrinsic_state_create_info const &);

    Cascaded_shadow_map::Intrinsic_state const *cascaded_shadow_map_intrinsic_state() const noexcept {
      return &_cascaded_shadow_map_intrinsic_state;
    }

    constexpr std::uint32_t surface_shader_program() const noexcept {
      return _surface_shader_program.get();
    }

    constexpr std::uint32_t lighting_shader_program() const noexcept {
      return _lighting_shader_program.get();
    }

    constexpr std::uint32_t temporal_antialiasing_shader_program() const noexcept {
      return _temporal_antialiasing_shader_program.get();
    }

    constexpr std::uint32_t postprocessing_shader_program() const noexcept {
      return _postprocessing_shader_program.get();
    }

    constexpr std::uint32_t empty_vertex_array() const noexcept {
      return _empty_vertex_array.get();
    }

  private:
    Cascaded_shadow_map::Intrinsic_state _cascaded_shadow_map_intrinsic_state;
    wrappers::Unique_shader_program _surface_shader_program;
    wrappers::Unique_shader_program _lighting_shader_program;
    wrappers::Unique_shader_program _temporal_antialiasing_shader_program;
    wrappers::Unique_shader_program _postprocessing_shader_program;
    wrappers::Unique_vertex_array _empty_vertex_array;
  };

  explicit Render_stream(Intrinsic_state const *intrinsic_state, Render_stream_create_info const &create_info) noexcept;

  void render() final;

  Render_target *target() const noexcept final {
    return _target;
  }

  Scene const *scene() const noexcept final {
    return _scene;
  }

  Camera const *camera() const noexcept final {
    return _camera;
  }

private:
  using Clock = std::chrono::system_clock;

  void acquire_surface_resource();

  void release_surface_resource();

  void write_surface_resource(math::Mat4x4f const &view_projection_matrix);

  void acquire_cascaded_shadow_map();

  void release_cascaded_shadow_map();

  void draw_cascaded_shadow_map();

  void draw_visibility_buffer(math::Mat4x4f const &jitter_matrix);

  void draw_surfaces(math::Mat4x4f const &jitter_matrix);

  // void draw_wireframes(math::Mat4x4f const &view_projection_matrix);

  void do_lighting(math::Mat4x4f const &inverse_view_matrix);

  void do_temporal_antialiasing(float blend_factor);

  void do_postprocessing();

  Intrinsic_state const *_intrinsic_state;
  Render_target *_target;
  Scene const *_scene;
  Camera const *_camera;
  Surface_resource _surface_resource;
  Cascaded_shadow_map _cascaded_shadow_map;
  Visibility_buffer _visibility_buffer;
  Triple_buffer<Uniform_buffer> _lighting_uniform_buffer;
  Temporal_antialiasing_resource _taa_resource;
  std::uint32_t _frame_number{};
  std::optional<std::chrono::time_point<Clock>> _frame_time;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
