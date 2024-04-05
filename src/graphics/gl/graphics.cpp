#include "graphics.h"

#include <cassert>

#include <array>
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

#include "scene.h"
#include "surface_material.h"
#include "surface_mesh.h"
#include "texture.h"
#include "unique_shader_handle.h"

namespace marlon {
namespace graphics {
namespace {
constexpr auto surface_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;
layout(location = 1) in vec3 model_space_normal;
layout(location = 2) in vec2 texcoord;

out Vertex_data {
  vec3 view_space_position;
  vec3 view_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) uniform mat4 model_view_matrix;
layout(location = 1) uniform mat4 model_view_clip_matrix;

void main() {
  vertex_data.view_space_position = (model_view_matrix * vec4(model_space_position, 1.0)).xyz;
  vertex_data.view_space_normal = (model_view_matrix * vec4(model_space_normal, 0.0)).xyz;
  vertex_data.texcoord = texcoord;
  gl_Position = model_view_clip_matrix * vec4(model_space_position, 1.0);
}
)";

constexpr auto surface_fragment_shader_source = R"(
#version 460 core

in Vertex_data {
  vec3 view_space_position;
  vec3 view_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) out vec4 out_color;

layout(binding = 0) uniform sampler2D base_color_texture;
layout(location = 2) uniform vec3 base_color_tint;

float luminance(vec3 v) {
  return dot(v, vec3(0.2126, 0.7152, 0.0722));
}

vec3 tonemap(vec3 v) {
  float l = luminance(v);
  vec3 tv = v / (v + vec3(1.0));
  return mix(v / (l + vec3(1.0)), tv, tv);
}

void main() {
  vec3 base_color = texture(base_color_texture, vertex_data.texcoord).rgb * base_color_tint;
  vec3 n = normalize(vertex_data.view_space_normal);
  vec3 l = normalize(vec3(-1.0, 1.0, 1.0));
  out_color = vec4(base_color * (max(dot(n, l), 0.0) * 0.9 + 0.1), 1.0);
}
)";

constexpr auto wireframe_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;

layout(location = 0) uniform mat4 model_view_clip_matrix;

void main() {
  gl_Position = model_view_clip_matrix * vec4(model_space_position, 1.0);
}
)";

constexpr auto wireframe_fragment_shader_source = R"(
#version 460 core

layout(location = 0) out vec4 out_color;

layout(location = 1) uniform vec3 in_color;

void main() {
  out_color = vec4(in_color, 1.0);
}
)";

void compile_shader(GLuint shader) {
  GLint status;
  glCompileShader(shader);
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(shader, log_size, nullptr, log.data());
    throw std::runtime_error{log.data()};
  }
}

void link_shader_program(GLuint shader_program) {
  GLint status;
  glLinkProgram(shader_program);
  glGetProgramiv(shader_program, GL_LINK_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size;
    glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetProgramInfoLog(shader_program, log_size, nullptr, log.data());
    throw std::runtime_error{log.data()};
  }
}

Gl_unique_shader_program_handle
make_shader_program(char const *vertex_shader_source,
                    char const *fragment_shader_source) {
  auto const vertex_shader{gl_make_unique_shader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader.get(), 1, &vertex_shader_source, nullptr);
  compile_shader(vertex_shader.get());
  auto const fragment_shader{gl_make_unique_shader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader.get(), 1, &fragment_shader_source, nullptr);
  compile_shader(fragment_shader.get());
  auto result = gl_make_unique_shader_program();
  glAttachShader(result.get(), vertex_shader.get());
  glAttachShader(result.get(), fragment_shader.get());
  link_shader_program(result.get());
  glDetachShader(result.get(), vertex_shader.get());
  glDetachShader(result.get(), fragment_shader.get());
  return result;
}

Gl_unique_texture_handle make_default_base_color_texture() {
  auto result = gl_make_unique_texture(GL_TEXTURE_2D);
  auto const pixels = std::array<std::uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF};
  glTextureStorage2D(result.get(), 1, GL_RGBA8, 1, 1);
  glTextureSubImage2D(
      result.get(), 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  return result;
}
} // namespace

Gl_graphics::Gl_graphics(Gl_graphics_create_info const &create_info) {
  if (gladLoadGL(create_info.function_loader) == 0) {
    throw std::runtime_error{"Failed to load OpenGL functions."};
  }
  _default_render_target = std::make_unique<Gl_default_render_target>(
      Gl_default_render_target_create_info{
          .window = create_info.window,
      });
  _surface_shader_program = make_shader_program(surface_vertex_shader_source,
                                                surface_fragment_shader_source);
  _wireframe_shader_program = make_shader_program(
      wireframe_vertex_shader_source, wireframe_fragment_shader_source);
  _default_base_color_texture = make_default_base_color_texture();
}

Render_target *Gl_graphics::get_default_render_target() noexcept {
  return _default_render_target.get();
}

void Gl_graphics::destroy_render_target(Render_target *) noexcept {
  // delete static_cast<Gl_render_target *>(target);
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

namespace {
math::Mat3x4f calculate_view_matrix(math::Vec3f const &position,
                                    math::Quatf const &orientation) {
  auto const upper_left_inv =
      math::Mat3x3f{{(1.0f - 2.0f * orientation.v.y * orientation.v.y -
                      2.0f * orientation.v.z * orientation.v.z),
                     (2.0f * orientation.v.x * orientation.v.y +
                      2.0f * orientation.w * orientation.v.z),
                     (2.0f * orientation.v.x * orientation.v.z -
                      2.0f * orientation.w * orientation.v.y)},
                    {(2.0f * orientation.v.x * orientation.v.y -
                      2.0f * orientation.w * orientation.v.z),
                     (1.0f - 2.0f * orientation.v.x * orientation.v.x -
                      2.0f * orientation.v.z * orientation.v.z),
                     (2.0f * orientation.v.y * orientation.v.z +
                      2.0f * orientation.w * orientation.v.x)},
                    {(2.0f * orientation.v.x * orientation.v.z +
                      2.0f * orientation.w * orientation.v.y),
                     (2.0f * orientation.v.y * orientation.v.z -
                      2.0f * orientation.w * orientation.v.x),
                     (1.0f - 2.0f * orientation.v.x * orientation.v.x -
                      2.0f * orientation.v.y * orientation.v.y)}};
  return math::Mat3x4f{{upper_left_inv[0][0],
                        upper_left_inv[0][1],
                        upper_left_inv[0][2],
                        -(upper_left_inv[0] * position)},
                       {upper_left_inv[1][0],
                        upper_left_inv[1][1],
                        upper_left_inv[1][2],
                        -(upper_left_inv[1] * position)},
                       {upper_left_inv[2][0],
                        upper_left_inv[2][1],
                        upper_left_inv[2][2],
                        -(upper_left_inv[2] * position)}};
}

math::Mat4x4f calculate_clip_matrix(math::Vec2f const &zoom,
                                    float near_plane_distance,
                                    float far_plane_distance) {
  return math::Mat4x4f{{zoom.x, 0.0f, 0.0f, 0.0f},
                       {0.0f, zoom.y, 0.0f, 0.0f},
                       {0.0f,
                        0.0f,
                        -(far_plane_distance + near_plane_distance) /
                            (far_plane_distance - near_plane_distance),
                        -2.0f * near_plane_distance * far_plane_distance /
                            (far_plane_distance - near_plane_distance)},
                       {0.0f, 0.0f, -1.0f, 0.0f}};
}
} // namespace

void Gl_graphics::render(Render_info const &info) {
  auto const gl_source_scene = static_cast<Gl_scene *>(info.source);
  auto const gl_target = static_cast<Gl_render_target *>(info.target);
  auto const view_matrix_3x4 =
      calculate_view_matrix(info.position, info.orientation);
  auto const view_matrix_4x4 =
      math::Mat4x4f{view_matrix_3x4, {0.0f, 0.0f, 0.0f, 1.0f}};
  auto const clip_matrix = calculate_clip_matrix(
      info.zoom, info.near_plane_distance, info.far_plane_distance);
  glBindFramebuffer(GL_FRAMEBUFFER, gl_target->get_framebuffer());
  auto const viewport_extents = gl_target->get_extents();
  glViewport(0, 0, viewport_extents.x, viewport_extents.y);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glEnable(GL_POLYGON_OFFSET_FILL);
  // glPolygonOffset(1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  // glDisable(GL_BLEND);
  glEnable(GL_FRAMEBUFFER_SRGB);
  glUseProgram(_surface_shader_program.get());
  gl_source_scene->draw_surfaces(_surface_shader_program.get(),
                                 _default_base_color_texture.get(),
                                 0,
                                 1,
                                 2,
                                 view_matrix_4x4,
                                 clip_matrix * view_matrix_4x4);
  glEnable(GL_POLYGON_OFFSET_LINE);
  glPolygonOffset(-1.0f, -1.0f);
  glLineWidth(2.0f);
  glDepthFunc(GL_LEQUAL);
  // glEnable(GL_BLEND);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glEnable(GL_LINE_SMOOTH);
  glUseProgram(_wireframe_shader_program.get());
  gl_source_scene->draw_wireframes(
      _wireframe_shader_program.get(), 0, 1, clip_matrix * view_matrix_4x4);
}
} // namespace graphics
} // namespace marlon