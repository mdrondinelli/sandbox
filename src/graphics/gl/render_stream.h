#ifndef MARLON_GL_RENDER_STREAM_H
#define MARLON_GL_RENDER_STREAM_H

#include "../render_stream.h"
#include "render_target.h"
#include "scene.h"
#include "unique_shader_program_handle.h"
#include "unique_texture_handle.h"

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

      constexpr std::uint32_t get_surface_shader_program() const noexcept {
        return _surface_shader_program.get();
      }

      constexpr std::uint32_t get_wireframe_shader_program() const noexcept {
        return _wireframe_shader_program.get();
      }

      constexpr std::uint32_t get_default_base_color_texture() const noexcept {
        return _default_base_color_texture.get();
      }

    private:
      Unique_shader_program_handle _surface_shader_program;
      Unique_shader_program_handle _wireframe_shader_program;
      Unique_texture_handle _default_base_color_texture;
    };

    explicit Render_stream(
        Intrinsic_state const *intrinsic_state,
        Render_stream_create_info const &create_info) noexcept
        : _intrinsic_state{intrinsic_state},
          _target{static_cast<Render_target *>(create_info.target)},
          _scene{static_cast<Scene const *>(create_info.scene)},
          _camera{create_info.camera} {}

    void render() final;

    Render_target *get_target() const noexcept final { return _target; }

    Scene const *get_scene() const noexcept final { return _scene; }

    Camera const *get_camera() const noexcept final { return _camera; }

  private:
    Intrinsic_state const *_intrinsic_state;
    Render_target *_target;
    Scene const *_scene;
    Camera const *_camera;
  };
}
} // namespace graphics
} // namespace marlon

#endif