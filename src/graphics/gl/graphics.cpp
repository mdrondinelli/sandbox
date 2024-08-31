#include "graphics.h"

#include <cassert>

#include <stdexcept>

#include <glad/gl.h>

#include "render_stream.h"
#include "surface_mesh.h"

using namespace marlon::math;

namespace marlon::graphics::gl {
Graphics::Graphics(Graphics_create_info const &create_info) {
  if (gladLoadGL(create_info.loader) == 0) {
    throw std::runtime_error{"Failed to load OpenGL functions."};
  }
  _default_render_target = Default_render_target{{
      .window = create_info.window,
  }};
  _render_stream_intrinsic_state = Render_stream::Intrinsic_state{{}};
}

Render_target *Graphics::get_default_render_target() noexcept {
  return &_default_render_target;
}

void Graphics::destroy_render_target(graphics::Render_target *render_target) noexcept {
  delete static_cast<Render_target *>(render_target);
}

Render_stream *Graphics::create_render_stream(Render_stream_create_info const &create_info) {
  return new Render_stream{&_render_stream_intrinsic_state, create_info};
}

void Graphics::destroy_render_stream(graphics::Render_stream *render_stream) noexcept {
  delete static_cast<Render_stream *>(render_stream);
}

Surface_mesh *Graphics::create_surface_mesh(Surface_mesh_create_info const &create_info) {
  return new Surface_mesh{create_info};
}

void Graphics::destroy_surface_mesh(graphics::Surface_mesh *surface_mesh) noexcept {
  delete static_cast<Surface_mesh *>(surface_mesh);
}
} // namespace marlon::graphics::gl
