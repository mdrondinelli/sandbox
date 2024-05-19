#ifndef MARLON_GL_RENDER_STREAM_H
#define MARLON_GL_RENDER_STREAM_H

#include "../render_stream.h"
#include "./wrappers/unique_shader_program.h"
#include "./wrappers/unique_texture.h"
#include "cascaded_shadow_map.h"
#include "surface_resource.h"
#include "render_target.h"

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

    Cascaded_shadow_map::Intrinsic_state const *
    cascaded_shadow_map_intrinsic_state() const noexcept {
      return &_cascaded_shadow_map_intrinsic_state;
    }

    constexpr std::uint32_t surface_shader_program() const noexcept {
      return _surface_shader_program.get();
    }

    constexpr std::uint32_t wireframe_shader_program() const noexcept {
      return _wireframe_shader_program.get();
    }

    constexpr std::uint32_t default_base_color_texture() const noexcept {
      return _default_base_color_texture.get();
    }

  private:
    Cascaded_shadow_map::Intrinsic_state _cascaded_shadow_map_intrinsic_state;
    wrappers::Unique_shader_program _surface_shader_program;
    wrappers::Unique_shader_program _wireframe_shader_program;
    wrappers::Unique_texture _default_base_color_texture;
  };

  explicit Render_stream(Intrinsic_state const *intrinsic_state,
                         Render_stream_create_info const &create_info) noexcept
      : _intrinsic_state{intrinsic_state},
        _target{static_cast<Render_target *>(create_info.target)},
        _scene{static_cast<Scene const *>(create_info.scene)},
        _camera{create_info.camera} {}

  void render() final;

  Render_target *target() const noexcept final { return _target; }

  Scene const *scene() const noexcept final { return _scene; }

  Camera const *camera() const noexcept final { return _camera; }

private:
  void prepare_surface_resource();

  void draw_csm();

  void draw_surfaces(math::Mat4x4f const &view_matrix,
                     math::Mat4x4f const &projection_matrix);

  void draw_wireframes(math::Mat4x4f const &view_projection_matrix);

  Intrinsic_state const *_intrinsic_state;
  Render_target *_target;
  Scene const *_scene;
  Camera const *_camera;
  Surface_resource _surface_resource;
  Cascaded_shadow_map _cascaded_shadow_map;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif