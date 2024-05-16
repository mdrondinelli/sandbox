#ifndef MARLON_GRAPHICS_GL_GRAPHICS_H
#define MARLON_GRAPHICS_GL_GRAPHICS_H

#include <memory>

#include "../graphics.h"
#include "default_render_target.h"
#include "render_stream.h"
#include "window.h"
#include "texture.h"
#include "surface_mesh.h"
#include "wireframe_mesh.h"
#include "scene.h"

namespace marlon {
namespace graphics {
namespace gl {
using Api_function = void (*)();
using Api_loader = Api_function (*)(char const *);

struct Graphics_create_info {
  Api_loader loader;
  Window const *window;
};

class Graphics final : public graphics::Graphics {
public:
  explicit Graphics(Graphics_create_info const &create_info);

  Graphics(Graphics const &other) = delete;

  Graphics &operator=(Graphics const &other) = delete;

  Texture *create_texture(Texture_create_info const &create_info) final;

  void destroy_texture(graphics::Texture *texture) noexcept final;

  Surface_mesh *
  create_surface_mesh(Surface_mesh_create_info const &create_info) final;

  void destroy_surface_mesh(graphics::Surface_mesh *surface_mesh) noexcept final;

  Wireframe_mesh *
  create_wireframe_mesh(Wireframe_mesh_create_info const &create_info) final;

  void destroy_wireframe_mesh(graphics::Wireframe_mesh *wireframe_mesh) noexcept final;

  Scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(graphics::Scene *scene) noexcept final;

  Render_target *get_default_render_target() noexcept;

  void destroy_render_target(graphics::Render_target *target) noexcept final;

  Render_stream *
  create_render_stream(Render_stream_create_info const &create_info) final;

  void destroy_render_stream(graphics::Render_stream *render_stream) noexcept final;

private:
  Default_render_target _default_render_target;
  Render_stream::Intrinsic_state _render_stream_intrinsic_state;
};   
}
} // namespace graphics
} // namespace marlon

#endif