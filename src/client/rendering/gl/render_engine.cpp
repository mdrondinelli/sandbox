#include "render_engine.h"

#include <iostream>

#include <glad/glad.h>

#include "unique_shader.h"

namespace marlon {
namespace rendering {
namespace {
constexpr auto vert_src = R"(#version 460 core
layout(location = 0) in vec3 position;

layout(location = 0) uniform mat4 model_view_projection;

void main() {
  gl_Position = model_view_projection * vec4(position, 1.0);
}
)";

constexpr auto frag_src = R"(#version 460 core
layout(location = 0) out vec4 fragColor;

void main() {
  fragColor = vec4(1.0f, 1.0f, 1.0f, 1.0);
}
)";
} // namespace

Gl_render_engine::Gl_render_engine()
    : _default_render_destination{std::make_unique<
          Gl_default_render_destination>()},
      _shader_program{make_gl_unique_shader_program()} {
  GLint status;
  auto const vertex_shader{make_gl_unique_shader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader.get(), 1, &vert_src, nullptr);
  glCompileShader(vertex_shader.get());
  glGetShaderiv(vertex_shader.get(), GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size = 0;
    glGetShaderiv(vertex_shader.get(), GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(vertex_shader.get(), log_size, nullptr, log.data());
    std::cout << log.data() << std::endl;
    throw;
  }
  auto const fragment_shader{make_gl_unique_shader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader.get(), 1, &frag_src, nullptr);
  glCompileShader(fragment_shader.get());
  glGetShaderiv(fragment_shader.get(), GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size;
    glGetShaderiv(fragment_shader.get(), GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(fragment_shader.get(), log_size, nullptr, log.data());
    std::cout << log.data() << std::endl;
    throw;
  }
  glAttachShader(_shader_program.get(), vertex_shader.get());
  glAttachShader(_shader_program.get(), fragment_shader.get());
  glLinkProgram(_shader_program.get());
  glDetachShader(_shader_program.get(), vertex_shader.get());
  glDetachShader(_shader_program.get(), fragment_shader.get());
  glGetProgramiv(_shader_program.get(), GL_LINK_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size;
    glGetProgramiv(_shader_program.get(), GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetProgramInfoLog(_shader_program.get(), log_size, nullptr, log.data());
    std::cout << log.data() << std::endl;
    throw;
  }
}

Gl_mesh *Gl_render_engine::create_mesh(Mesh_create_info const &create_info) {
  return new Gl_mesh{create_info};
}

void Gl_render_engine::destroy_mesh(Mesh *mesh) {
  delete static_cast<Gl_mesh *>(mesh);
}

Gl_material *
Gl_render_engine::create_material(Material_create_info const &create_info) {
  return new Gl_material{create_info};
}

void Gl_render_engine::destroy_material(Material *material) {
  delete static_cast<Gl_material *>(material);
}

Gl_surface *
Gl_render_engine::create_surface(Surface_create_info const &create_info) {
  return new Gl_surface{create_info};
}

void Gl_render_engine::destroy_surface(Surface *surface) {
  delete static_cast<Gl_surface *>(surface);
}

Gl_scene *Gl_render_engine::create_scene(Scene_create_info const &create_info) {
  return new Gl_scene{create_info};
}

void Gl_render_engine::destroy_scene(Scene *scene) {
  delete static_cast<Gl_scene *>(scene);
}

Gl_scene_diff *
Gl_render_engine::create_scene_diff(Scene_diff_create_info const &create_info) {
  return new Gl_scene_diff{create_info};
}

void Gl_render_engine::destroy_scene_diff(Scene_diff *scene_diff) noexcept {
  delete static_cast<Gl_scene_diff *>(scene_diff);
}

void Gl_render_engine::apply_scene_diff(Scene_diff *scene_diff) {
  static_cast<Gl_scene_diff *>(scene_diff)->_impl.apply();
}

void Gl_render_engine::apply_scene_diff(Scene_diff *scene_diff, float factor) {
  static_cast<Gl_scene_diff *>(scene_diff)->_impl.apply(factor);
}

Gl_scene_node *Gl_render_engine::record_scene_node_creation(
    Scene_diff *scene_diff, Scene_node_create_info const &create_info) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_creation(create_info);
}

void Gl_render_engine::record_scene_node_destruction(Scene_diff *scene_diff,
                                                     Scene_node *scene_node) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_destruction(
          static_cast<Gl_scene_node *>(scene_node));
}

void Gl_render_engine::record_scene_node_translation_continuous(
    Scene_diff *scene_diff, Scene_node *scene_node, math::Vec3f const &value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_translation_continuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_render_engine::record_scene_node_translation_discontinuous(
    Scene_diff *scene_diff, Scene_node *scene_node, math::Vec3f const &value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_translation_discontinuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_render_engine::record_scene_node_rotation_continuous(
    Scene_diff *scene_diff, Scene_node *scene_node, math::Quatf const &value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_rotation_continuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_render_engine::record_scene_node_rotation_discontinuous(
    Scene_diff *scene_diff, Scene_node *scene_node, math::Quatf const &value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_rotation_discontinuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_render_engine::record_scene_node_scale_continuous(
    Scene_diff *scene_diff, Scene_node *scene_node, float value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_scale_continuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_render_engine::record_scene_node_scale_discontinuous(
    Scene_diff *scene_diff, Scene_node *scene_node, float value) {
  static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_scene_node_scale_discontinuous(
          static_cast<Gl_scene_node *>(scene_node), value);
}

Gl_camera *Gl_render_engine::record_camera_creation(
    Scene_diff *scene_diff, Camera_create_info const &create_info) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_camera_creation(create_info);
}

void Gl_render_engine::record_camera_destruction(Scene_diff *scene_diff,
                                                 Camera *camera) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_camera_destruction(static_cast<Gl_camera *>(camera));
}

Gl_camera_instance *Gl_render_engine::record_camera_instance_creation(
    Scene_diff *scene_diff, Camera_instance_create_info const &create_info) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_camera_instance_creation(create_info);
}

void Gl_render_engine::record_camera_instance_destruction(
    Scene_diff *scene_diff, Camera_instance *camera_instance) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_camera_instance_destruction(
          static_cast<Gl_camera_instance *>(camera_instance));
}

Gl_surface_instance *Gl_render_engine::record_surface_instance_creation(
    Scene_diff *scene_diff, Surface_instance_create_info const &create_info) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_surface_instance_creation(create_info);
}

void Gl_render_engine::record_surface_instance_destruction(
    Scene_diff *scene_diff, Surface_instance *surface_instance) {
  return static_cast<Gl_scene_diff *>(scene_diff)
      ->_impl.record_surface_instance_destruction(
          static_cast<Gl_surface_instance *>(surface_instance));
}

Gl_default_render_destination *
Gl_render_engine::get_default_render_destination() noexcept {
  return _default_render_destination.get();
}

void Gl_render_engine::destroy_render_destination(
    Render_destination *destination) {
  delete static_cast<Gl_render_destination *>(destination);
}

Gl_render_stream *Gl_render_engine::create_render_stream(
    Render_stream_create_info const &create_info) {
  return new Gl_render_stream{create_info};
}

void Gl_render_engine::destroy_render_stream(Render_stream *stream) {
  delete stream;
}

void Gl_render_engine::render(Render_stream *stream) {
  static_cast<Gl_render_stream *>(stream)->_impl.render(_shader_program.get(),
                                                        0);
}
} // namespace rendering
} // namespace marlon