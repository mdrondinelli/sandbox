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

#include "material.h"
#include "mesh.h"
#include "scene.h"
#include "surface.h"
#include "texture.h"
#include "unique_shader_handle.h"

namespace marlon {
namespace graphics {
namespace {
constexpr auto vert_src = R"(#version 460 core
layout(location = 0) in vec3 model_space_position;
layout(location = 1) in vec2 texcoord;

out Vertex_data {
  vec3 view_space_position;
  vec2 texcoord;
} vertex_data;

layout(location = 0) uniform mat4 model_view_matrix;
layout(location = 1) uniform mat4 model_view_clip_matrix;

void main() {
  vertex_data.view_space_position = (model_view_matrix * vec4(model_space_position, 1.0)).xyz;
  vertex_data.texcoord = texcoord;
  gl_Position = model_view_clip_matrix * vec4(model_space_position, 1.0);
}
)";

constexpr auto frag_src = R"(#version 460 core
in Vertex_data {
  vec3 view_space_position;
  vec2 texcoord;
} vertex_data;

layout(location = 0) out vec4 fragColor;

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
  vec3 n = normalize(cross(dFdx(vertex_data.view_space_position), dFdy(vertex_data.view_space_position)));
  vec3 l = normalize(vec3(-1.0, 1.0, 1.0));
  fragColor = vec4(base_color * (max(dot(n, l), 0.0) * 0.9 + 0.1), 1.0);
}
)";
} // namespace

Gl_graphics::Gl_graphics(Gl_graphics_create_info const &create_info) {
  if (gladLoadGL(create_info.function_loader) == 0) {
    throw std::runtime_error{"Failed to load OpenGL functions."};
  } 
  _default_render_target = std::make_unique<Gl_default_render_target>(
      Gl_default_render_target_create_info{
          .window = create_info.window,
      });
  _shader_program = gl_make_unique_shader_program();
  GLint status;
  auto const vertex_shader{gl_make_unique_shader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader.get(), 1, &vert_src, nullptr);
  glCompileShader(vertex_shader.get());
  glGetShaderiv(vertex_shader.get(), GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size = 0;
    glGetShaderiv(vertex_shader.get(), GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(vertex_shader.get(), log_size, nullptr, log.data());
    throw std::runtime_error{log.data()};
  }
  auto const fragment_shader{gl_make_unique_shader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader.get(), 1, &frag_src, nullptr);
  glCompileShader(fragment_shader.get());
  glGetShaderiv(fragment_shader.get(), GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    GLint log_size;
    glGetShaderiv(fragment_shader.get(), GL_INFO_LOG_LENGTH, &log_size);
    std::vector<char> log;
    log.resize(log_size);
    glGetShaderInfoLog(fragment_shader.get(), log_size, nullptr, log.data());
    throw std::runtime_error{log.data()};
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
    throw std::runtime_error{log.data()};
  }
  _default_base_color_texture = gl_make_unique_texture(GL_TEXTURE_2D);
  auto const default_base_color_texture_pixels =
      std::array<std::uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF};
  glTextureStorage2D(_default_base_color_texture.get(), 1, GL_RGBA8, 1, 1);
  glTextureSubImage2D(_default_base_color_texture.get(),
                      0,
                      0,
                      0,
                      1,
                      1,
                      GL_RGBA,
                      GL_UNSIGNED_BYTE,
                      default_base_color_texture_pixels.data());
}

Mesh *Gl_graphics::create_mesh(Mesh_create_info const &create_info) {
  return new Gl_mesh{create_info};
}

void Gl_graphics::destroy_mesh(Mesh *mesh) noexcept {
  delete static_cast<Gl_mesh *>(mesh);
}

Texture *Gl_graphics::create_texture(Texture_create_info const &create_info) {
  return new Gl_texture{create_info};
}

void Gl_graphics::destroy_texture(Texture *texture) noexcept {
  delete static_cast<Gl_texture *>(texture);
}

Material *
Gl_graphics::create_material(Material_create_info const &create_info) {
  return new Gl_material{create_info};
}

void Gl_graphics::destroy_material(Material *material) noexcept {
  delete static_cast<Gl_material *>(material);
}

Scene *Gl_graphics::create_scene(Scene_create_info const &create_info) {
  return new Gl_scene{create_info};
}

void Gl_graphics::destroy_scene(Scene *scene) noexcept {
  delete static_cast<Gl_scene *>(scene);
}

Surface *Gl_graphics::create_surface(Surface_create_info const &create_info) {
  return new Gl_surface{create_info};
}

void Gl_graphics::destroy_surface(Surface *surface) noexcept {
  delete static_cast<Gl_surface *>(surface);
}

Render_target *Gl_graphics::get_default_render_target() noexcept {
  return _default_render_target.get();
}

void Gl_graphics::destroy_render_target(Render_target *) noexcept {
  // delete static_cast<Gl_render_target *>(target);
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
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_FRAMEBUFFER_SRGB);
  glBindFramebuffer(GL_FRAMEBUFFER, gl_target->get_framebuffer());
  auto const viewport_extents = gl_target->get_extents();
  glViewport(0, 0, viewport_extents.x, viewport_extents.y);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(_shader_program.get());
  gl_source_scene->draw_surfaces(_shader_program.get(),
                                 _default_base_color_texture.get(),
                                 0,
                                 1,
                                 2,
                                 view_matrix_4x4,
                                 clip_matrix * view_matrix_4x4);
}
} // namespace graphics
} // namespace marlon