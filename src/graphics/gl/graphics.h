#ifndef MARLON_GRAPHICS_GL_GRAPHICS_H
#define MARLON_GRAPHICS_GL_GRAPHICS_H

#include "../graphics.h"
#include "cascaded_shadow_map.h"
#include "default_render_target.h"
#include "render_stream.h"
#include "surface_mesh.h"
#include "window.h"

namespace marlon {
namespace graphics {
namespace gl {
using Api_function = void (*)();
using Api_loader = Api_function (*)(char const *);

struct Graphics_create_info {
  Api_loader loader;
  Window *window;
};

class Graphics final : public graphics::Graphics {
public:
  explicit Graphics(Graphics_create_info const &create_info);

  Graphics(Graphics const &other) = delete;

  Graphics &operator=(Graphics const &other) = delete;

  Graphics_implementation_info const &implementation_info() const noexcept final {
    return _implementation_info;
  }

  Surface_mesh *create_surface_mesh(Surface_mesh_create_info const &create_info) final;

  void destroy_surface_mesh(graphics::Surface_mesh *surface_mesh) noexcept final;

  Render_target *get_default_render_target() noexcept;

  void destroy_render_target(graphics::Render_target *target) noexcept final;

  Render_stream *create_render_stream(Render_stream_create_info const &create_info) final;

  void destroy_render_stream(graphics::Render_stream *render_stream) noexcept final;

private:
  Graphics_implementation_info _implementation_info{
      .cascaded_shadow_map_max_cascade_count = Cascaded_shadow_map::max_cascade_count,
  };
  Default_render_target _default_render_target;
  Render_stream::Intrinsic_state _render_stream_intrinsic_state;
};
} // namespace gl
} // namespace graphics
} // namespace marlon

#endif
