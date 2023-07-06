#include "renderer.h"

namespace marlon {
namespace rendering {
namespace {
constexpr auto vert_src = R"(#version 460 core
layout (location = 0) in vec3 position;

uniform mat4 model_view_projection;

void main() {
  gl_Position = model_view_projection * vec4(position, 1.0);
}
)";

constexpr auto vert_src = R"(#version 460 core
layout (location = 0) out vec4 fragColor;

uniform vec3 albedo;

void main() {
  fragColor = vec4(albedo, 1.0);
}
)";
} // namespace

Gl_renderer::Gl_renderer()
    : _default_render_destination{std::make_unique<Render_destination>()} {}

Gl_camera *Gl_renderer::create_camera(Camera_create_info const &create_info) {
  return new Gl_camera{create_info};
}

void Gl_renderer::destroy_camera(Camera *camera) { delete camera; }

Gl_mesh *Gl_renderer::create_mesh(Mesh_create_info const &create_info) {
  return new Gl_mesh{create_info};
}

void Gl_renderer::destroy_mesh(Mesh *mesh) { delete mesh; }

Gl_material *
Gl_renderer::create_material(Material_create_info const &create_info) {
  return new Gl_material{create_info};
}

void Gl_renderer::destroy_material(Material *material) { delete material; }

Gl_surface *
Gl_renderer::create_surface(Surface_create_info const &create_info) {
  return new Gl_surface{create_info};
}

void Gl_renderer::destroy_surface(Surface *surface) { delete surface; }

Gl_scene *Gl_renderer::create_scene(Scene_create_info const &) {
  return new Gl_scene;
}

void Gl_renderer::destroy_scene(Scene *scene) { delete scene; }

Render_destination *Gl_renderer::create_default_render_destination() noexcept {
  return _default_render_destination.get();
}

void Gl_renderer::destroy_render_destination(Render_destination *destination) {
  if (destination != _default_render_destination.get()) {
    delete destination;
  }
}
} // namespace rendering
} // namespace marlon