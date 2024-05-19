#include "render_stream.h"

#include <iostream>

#include <glad/gl.h>

#include <math/math.h>

#include "render_target.h"
#include "surface_mesh.h"
#include "texture.h"
#include "wireframe_mesh.h"
#include "wrappers/unique_shader.h"

namespace marlon {
namespace graphics {
namespace gl {
using namespace math;

namespace {
auto constexpr shadow_map_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;

layout(location = 0) uniform mat4 model_view_clip_matrix;

void main() {
  gl_Position = model_view_clip_matrix * vec4(model_space_position, 1.0);
}
)";

auto constexpr shadow_map_fragment_shader_source = R"(
#version 460 core

void main() {
}
)";

constexpr auto surface_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;
layout(location = 1) in vec3 model_space_normal;
layout(location = 2) in vec2 texcoord;

out Vertex_data {
  vec3 world_space_position;
  vec3 view_space_position;
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) uniform mat4 model_matrix;
layout(location = 1) uniform mat4 model_view_matrix;
layout(location = 2) uniform mat4 model_view_clip_matrix;

void main() {
  vertex_data.world_space_position = (model_matrix * vec4(model_space_position, 1.0)).xyz;
  vertex_data.view_space_position = (model_view_matrix * vec4(model_space_position, 1.0)).xyz;
  vertex_data.world_space_normal = mat3(model_matrix) * model_space_normal;
  vertex_data.texcoord = texcoord;
  gl_Position = model_view_clip_matrix * vec4(model_space_position, 1.0);
}
)";

constexpr auto surface_fragment_shader_source = R"(
#version 460 core

in Vertex_data {
  vec3 world_space_position;
  vec3 view_space_position;
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) out vec4 out_color;

layout(binding = 0) uniform sampler2D base_color_texture;
layout(binding = 1) uniform sampler2DArrayShadow shadow_map;

struct Cascade {
  mat4x3 view_clip_matrix;
  float far_plane_distance;
  float pixel_length;
};

layout(row_major, std140, binding = 0) uniform Cascaded_shadow_map {
  Cascade cascades[8];
  int cascade_count;
} csm;

layout(location = 3) uniform vec3 base_color_tint;
layout(location = 4) uniform vec3 ambient_irradiance;
layout(location = 5) uniform vec3 directional_light_irradiance;
layout(location = 6) uniform vec3 directional_light_direction;
layout(location = 7) uniform float exposure;

int select_csm_cascade() {
  float z = -vertex_data.view_space_position.z;
  for (int i = 0; i < csm.cascade_count - 1; ++i) {
    if (z < csm.cascades[i].far_plane_distance) {
      return i;
    }
  }
  return csm.cascade_count - 1;
}

float calculate_csm_shadow_factor(float n_dot_l) {
  int i = select_csm_cascade();
  vec3 p_world = vertex_data.world_space_position + directional_light_direction * tan(acos(n_dot_l)) * csm.cascades[i].pixel_length;
  vec3 p_clip = csm.cascades[i].view_clip_matrix * vec4(p_world, 1.0);
  vec4 texcoord = vec4(p_clip.xy * vec2(0.5, -0.5) + 0.5, i, clamp(p_clip.z, 0.0, 1.0));
  return texture(shadow_map, texcoord);
}

vec3 tonemap(vec3 v) {
  v = mat3(
    vec3(0.59719, 0.07600, 0.02840),
    vec3(0.35458, 0.90834, 0.13383),
    vec3(0.04823, 0.01566, 0.83777)
  ) * v;
  v = (v * (v + 0.0245786) - 0.000090537) / (v * (0.983729 * v + 0.4329510) + 0.238081);
  v = mat3(
    vec3(1.60475, -0.10208, -0.00327),
    vec3(-0.53108,  1.10813, -0.07276),
    vec3(-0.07367, -0.00605,  1.07602)
  ) * v;
  return clamp(v, vec3(0.0), vec3(1.0));
}

void main() {
  vec3 base_color = texture(base_color_texture, vertex_data.texcoord).rgb * base_color_tint;
  vec3 n = normalize(vertex_data.world_space_normal);
  vec3 l = directional_light_direction;
  float n_dot_l = dot(n, l);
  float shadow_factor = calculate_csm_shadow_factor(n_dot_l);
  // float shadow_factor = step(shadow_map_value, directional_light_clip_space_position.z);
  vec3 irradiance = ambient_irradiance + directional_light_irradiance * max(n_dot_l, 0.0) * shadow_factor;
  out_color = vec4(tonemap(irradiance * base_color * exposure), 1.0);
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
    std::cerr << log.data() << std::endl;
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
    std::cerr << log.data() << std::endl;
    throw std::runtime_error{log.data()};
  }
}

wrappers::Unique_shader_program
make_shader_program(char const *vertex_shader_source,
                    char const *fragment_shader_source) {
  auto const vertex_shader{wrappers::make_unique_shader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader.get(), 1, &vertex_shader_source, nullptr);
  compile_shader(vertex_shader.get());
  auto const fragment_shader{wrappers::make_unique_shader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader.get(), 1, &fragment_shader_source, nullptr);
  compile_shader(fragment_shader.get());
  auto result = wrappers::make_unique_shader_program();
  glAttachShader(result.get(), vertex_shader.get());
  glAttachShader(result.get(), fragment_shader.get());
  link_shader_program(result.get());
  glDetachShader(result.get(), vertex_shader.get());
  glDetachShader(result.get(), fragment_shader.get());
  return result;
}

wrappers::Unique_texture make_default_base_color_texture() {
  auto result = wrappers::make_unique_texture(GL_TEXTURE_2D);
  auto const pixels = std::array<std::uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF};
  glTextureStorage2D(result.get(), 1, GL_RGBA8, 1, 1);
  glTextureSubImage2D(
      result.get(), 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  return result;
}

Mat4x4f calculate_clip_matrix(Vec2f const &zoom, float near_plane_distance) {
  return Mat4x4f{{zoom.x, 0.0f, 0.0f, 0.0f},
                 {0.0f, -zoom.y, 0.0f, 0.0f},
                 {0.0f, 0.0f, 0.0f, near_plane_distance},
                 {0.0f, 0.0f, -1.0f, 0.0f}};
}
} // namespace

Render_stream::Intrinsic_state::Intrinsic_state(
    Intrinsic_state_create_info const &)
    : _shadow_map_shader_program{make_shader_program(
          shadow_map_vertex_shader_source, shadow_map_fragment_shader_source)},
      _surface_shader_program{make_shader_program(
          surface_vertex_shader_source, surface_fragment_shader_source)},
      _wireframe_shader_program{make_shader_program(
          wireframe_vertex_shader_source, wireframe_fragment_shader_source)},
      _default_base_color_texture{make_default_base_color_texture()} {}

void Render_stream::render() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(0.0f);
  glClipControl(GL_UPPER_LEFT, GL_ZERO_TO_ONE);
  glEnable(GL_DEPTH_CLAMP);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CW);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glEnable(GL_FRAMEBUFFER_SRGB);
  draw_csm();
  auto const view_matrix =
      rigid_inverse(Mat4x4f::rigid(_camera->position, _camera->orientation));
  auto const clip_matrix =
      calculate_clip_matrix(_camera->zoom, _camera->near_plane_distance);
  auto const view_clip_matrix = clip_matrix * view_matrix;
  glBindFramebuffer(GL_FRAMEBUFFER, _target->get_framebuffer());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glEnable(GL_POLYGON_OFFSET_FILL);
  // glPolygonOffset(1.0f, 1.0f);
  auto const viewport_extents = _target->get_extents();
  glViewport(0, 0, viewport_extents.x, viewport_extents.y);
  // glDisable(GL_BLEND);
  glDisable(GL_DEPTH_CLAMP);
  draw_surfaces(view_matrix, view_clip_matrix);
  glEnable(GL_POLYGON_OFFSET_LINE);
  glPolygonOffset(1.0f, 1.0f);
  glLineWidth(2.0f);
  glDepthFunc(GL_GEQUAL);
  // glEnable(GL_BLEND);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glEnable(GL_LINE_SMOOTH);
  draw_wireframes(view_clip_matrix);
}

void Render_stream::draw_csm() {
  auto constexpr model_view_clip_matrix_location = 0;
  if (_camera->csm_cascade_count <= 0) {
    _csm = {};
  } else if (_csm.cascade_count() != _camera->csm_cascade_count ||
             _csm.texture_resolution() != _camera->csm_texture_resolution) {
    _csm = Cascaded_shadow_map{{
        .texture_resolution = _camera->csm_texture_resolution,
        .cascade_count = _camera->csm_cascade_count,
    }};
  }
  if (!_scene->directional_light() || _camera->csm_cascade_count <= 0) {
    return;
  }
  _csm.update_frusta(*_camera, _scene->directional_light()->direction);
  for (auto i = 0; i < _csm.cascade_count(); ++i) {
    glBindFramebuffer(GL_FRAMEBUFFER, _csm.cascades()[i].framebuffer());
    glClear(GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, _csm.texture_resolution(), _csm.texture_resolution());
    auto const shader_program = _intrinsic_state->shadow_map_shader_program();
    glUseProgram(shader_program);
    for (auto const surface : _scene->surfaces()) {
      if (!surface->shadow_casting) {
        continue;
      }
      auto const model_matrix =
          Mat4x4f{surface->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
      auto const model_view_clip_matrix =
          Mat4x4f{_csm.cascades()[i].view_clip_matrix(),
                  {0.0f, 0.0f, 0.0f, 1.0f}} *
          model_matrix;
      glProgramUniformMatrix4fv(shader_program,
                                model_view_clip_matrix_location,
                                1,
                                GL_TRUE,
                                &model_view_clip_matrix[0][0]);
      auto const mesh = static_cast<Surface_mesh const *>(surface->mesh);
      mesh->bind_vertex_array();
      mesh->draw();
    }
  }
}

void Render_stream::draw_surfaces(Mat4x4f const &view_matrix,
                                  Mat4x4f const &view_clip_matrix) {
  auto constexpr model_matrix_location = 0;
  auto constexpr model_view_matrix_location = 1;
  auto constexpr model_view_clip_matrix_location = 2;
  auto constexpr base_color_tint_location = 3;
  auto constexpr ambient_irradiance_location = 4;
  auto constexpr directional_light_irradiance_location = 5;
  auto constexpr directional_light_direction_location = 6;
  // auto constexpr directional_light_view_clip_matrix_location = 6;
  auto constexpr exposure_location = 7;
  auto const shader_program = _intrinsic_state->surface_shader_program();
  auto const ambient_irradiance = _scene->ambient_irradiance();
  auto const &directional_light = _scene->directional_light();
  auto const exposure = _camera->exposure;
  glUseProgram(shader_program);
  glProgramUniform3f(shader_program,
                     ambient_irradiance_location,
                     ambient_irradiance.r,
                     ambient_irradiance.g,
                     ambient_irradiance.b);
  if (directional_light) {
    if (_csm) {
      glBindTextureUnit(1, _csm.texture());
      _csm.acquire_uniform_buffer();
      _csm.update_uniform_buffer();
      glBindBufferBase(GL_UNIFORM_BUFFER, 0, _csm.uniform_buffer());
      // glProgramUniformMatrix4x3fv(
      //     shader_program,
      //     directional_light_view_clip_matrix_location,
      //     1,
      //     GL_TRUE,
      //     &_csm.cascades().front().view_clip_matrix()[0][0]);
    }
    glProgramUniform3f(shader_program,
                       directional_light_irradiance_location,
                       directional_light->irradiance.r,
                       directional_light->irradiance.g,
                       directional_light->irradiance.b);
    glProgramUniform3f(shader_program,
                       directional_light_direction_location,
                       directional_light->direction.x,
                       directional_light->direction.y,
                       directional_light->direction.z);
  } else {
    glProgramUniform3f(shader_program,
                       directional_light_irradiance_location,
                       0.0f,
                       0.0f,
                       0.0f);
    glProgramUniform3f(
        shader_program, directional_light_direction_location, 1.0f, 0.0f, 0.0f);
  }
  glProgramUniform1f(shader_program, exposure_location, exposure);
  for (auto const surface : _scene->surfaces()) {
    if (!surface->visible) {
      continue;
    }
    auto const model_matrix =
        Mat4x4f{surface->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const model_view_matrix = view_matrix * model_matrix;
    auto const model_view_clip_matrix = view_clip_matrix * model_matrix;
    glProgramUniformMatrix4fv(
        shader_program, model_matrix_location, 1, GL_TRUE, &model_matrix[0][0]);
    glProgramUniformMatrix4fv(shader_program,
                              model_view_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_matrix[0][0]);
    glProgramUniformMatrix4fv(shader_program,
                              model_view_clip_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_clip_matrix[0][0]);
    auto const &material = surface->material;
    if (auto const base_color_texture =
            static_cast<Texture *>(material.base_color_texture)) {
      glBindTextureUnit(0, base_color_texture->get());
    } else {
      glBindTextureUnit(0, _intrinsic_state->default_base_color_texture());
    }
    glProgramUniform3f(shader_program,
                       base_color_tint_location,
                       material.base_color_tint.r,
                       material.base_color_tint.g,
                       material.base_color_tint.b);
    auto const mesh = static_cast<Surface_mesh const *>(surface->mesh);
    mesh->bind_vertex_array();
    mesh->draw();
  }
  _csm.release_uniform_buffer();
}

void Render_stream::draw_wireframes(math::Mat4x4f const &view_clip_matrix) {
  auto constexpr model_view_clip_matrix_location = 0;
  auto constexpr color_location = 1;
  auto const shader_program = _intrinsic_state->wireframe_shader_program();
  glUseProgram(shader_program);
  for (auto const wireframe : _scene->wireframes()) {
    if (!wireframe->visible) {
      continue;
    }
    auto const model_matrix =
        Mat4x4f{wireframe->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const model_view_clip_matrix = view_clip_matrix * model_matrix;
    glProgramUniformMatrix4fv(shader_program,
                              model_view_clip_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_clip_matrix[0][0]);
    glProgramUniform3f(shader_program,
                       color_location,
                       wireframe->color.r,
                       wireframe->color.g,
                       wireframe->color.b);
    auto const mesh = static_cast<Wireframe_mesh const *>(wireframe->mesh);
    mesh->bind_vertex_array();
    mesh->draw();
  }
}
} // namespace gl
} // namespace graphics
} // namespace marlon