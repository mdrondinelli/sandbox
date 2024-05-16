#include "graphics.h"

#include <cassert>

#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

#include "render_stream.h"
#include "scene.h"
#include "surface_material.h"
#include "surface_mesh.h"
#include "texture.h"

using namespace marlon::math;

namespace marlon {
namespace graphics {
Gl_graphics::Gl_graphics(Gl_graphics_create_info const &create_info) {
  if (gladLoadGL(create_info.function_loader) == 0) {
    throw std::runtime_error{"Failed to load OpenGL functions."};
  }
  _default_render_target = Gl_default_render_target{{
      .window = create_info.window,
  }};
  _render_stream_intrinsic_state = Gl_render_stream::Intrinsic_state{{}};
}

Render_target *Gl_graphics::get_default_render_target() noexcept {
  return &_default_render_target;
}

void Gl_graphics::destroy_render_target(Render_target *render_target) noexcept {
  delete static_cast<Gl_render_target *>(render_target);
}

Render_stream *Gl_graphics::create_render_stream(
    Render_stream_create_info const &create_info) {
  return new Gl_render_stream{&_render_stream_intrinsic_state, create_info};
}

void Gl_graphics::destroy_render_stream(Render_stream *render_stream) noexcept {
  delete static_cast<Gl_render_stream *>(render_stream);
}

Scene *Gl_graphics::create_scene(Scene_create_info const &create_info) {
  return new Gl_scene{create_info};
}

void Gl_graphics::destroy_scene(Scene *scene) noexcept {
  delete static_cast<Gl_scene *>(scene);
}

Surface_material *Gl_graphics::create_surface_material(
    Surface_material_create_info const &create_info) {
  return new Gl_surface_material{create_info};
}

void Gl_graphics::destroy_surface_material(
    Surface_material *surface_material) noexcept {
  delete static_cast<Gl_surface_material *>(surface_material);
}

Surface_mesh *
Gl_graphics::create_surface_mesh(Surface_mesh_create_info const &create_info) {
  return new Gl_surface_mesh{create_info};
}

void Gl_graphics::destroy_surface_mesh(Surface_mesh *surface_mesh) noexcept {
  delete static_cast<Gl_surface_mesh *>(surface_mesh);
}

Wireframe_mesh *Gl_graphics::create_wireframe_mesh(
    Wireframe_mesh_create_info const &create_info) {
  return new Gl_wireframe_mesh{create_info};
}

void Gl_graphics::destroy_wireframe_mesh(
    Wireframe_mesh *wireframe_mesh) noexcept {
  delete static_cast<Gl_wireframe_mesh *>(wireframe_mesh);
}

Texture *Gl_graphics::create_texture(Texture_create_info const &create_info) {
  return new Gl_texture{create_info};
}

void Gl_graphics::destroy_texture(Texture *texture) noexcept {
  delete static_cast<Gl_texture *>(texture);
}
} // namespace graphics
} // namespace marlon