#ifndef MARLON_GRAPHICS_GL_GRAPHICS_H
#define MARLON_GRAPHICS_GL_GRAPHICS_H

#include <memory>

#include "../graphics.h"
#include "default_render_target.h"
#include "render_stream.h"
#include "window.h"

namespace marlon {
namespace graphics {
using Gl_function = void (*)();

using Gl_function_loader = Gl_function (*)(char const *);

struct Gl_graphics_create_info {
  Gl_function_loader function_loader;
  Gl_window const *window;
};

class Gl_graphics final : public Graphics {
public:
  explicit Gl_graphics(Gl_graphics_create_info const &create_info);

  Gl_graphics(Gl_graphics const &other) = delete;

  Gl_graphics &operator=(Gl_graphics const &other) = delete;

  Texture *create_texture(Texture_create_info const &create_info) final;

  void destroy_texture(Texture *texture) noexcept final;

  Surface_material *create_surface_material(
      Surface_material_create_info const &create_info) final;

  void
  destroy_surface_material(Surface_material *surface_material) noexcept final;

  Surface_mesh *
  create_surface_mesh(Surface_mesh_create_info const &create_info) final;

  void destroy_surface_mesh(Surface_mesh *surface_mesh) noexcept final;

  Wireframe_mesh *
  create_wireframe_mesh(Wireframe_mesh_create_info const &create_info) final;

  void destroy_wireframe_mesh(Wireframe_mesh *wireframe_mesh) noexcept final;

  Scene *create_scene(Scene_create_info const &create_info) final;

  void destroy_scene(Scene *scene) noexcept final;

  Render_target *get_default_render_target() noexcept;

  void destroy_render_target(Render_target *target) noexcept final;

  Render_stream *
  create_render_stream(Render_stream_create_info const &create_info) final;

  void destroy_render_stream(Render_stream *render_stream) noexcept final;

private:
  Gl_default_render_target _default_render_target;
  Gl_render_stream::Intrinsic_state _render_stream_intrinsic_state;
};
} // namespace graphics
} // namespace marlon

#endif