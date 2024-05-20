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
using namespace util;

namespace {
constexpr auto surface_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;
layout(location = 1) in vec3 model_space_normal;
layout(location = 2) in vec2 texcoord;

out Vertex_data {
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) uniform mat4 view_projection_matrix;

layout(row_major, std140, binding = 0) uniform Surface {
  mat4x3 model_matrix;
  vec4 base_color_tint;
} surface;

void main() {
  vertex_data.world_space_normal = mat3(surface.model_matrix) * model_space_normal;
  vertex_data.texcoord = texcoord;
  gl_Position = view_projection_matrix * vec4(surface.model_matrix * vec4(model_space_position, 1.0), 1.0);
}
)";

constexpr auto surface_fragment_shader_source = R"(
#version 460 core

#define MAX_CASCADE_COUNT 4

in Vertex_data {
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec2 out_normal;

vec2 oct_encode(vec3 v) {
  vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + abs(v.z)));
  return v.z <= 0.0 ? (1.0 - abs(p.yx)) * (2.0 * step(0.0, p) - 1.0) : p;
}

layout(binding = 0) uniform sampler2D base_color_texture;
// layout(binding = 1) uniform sampler2DArrayShadow shadow_map;

// struct Cascade {
//   mat4x3 view_clip_matrix;
//   float far_plane_distance;
//   float pixel_length;
// };

// layout(row_major, std140, binding = 0) uniform Cascaded_shadow_map {
//   Cascade cascades[MAX_CASCADE_COUNT];
//   int cascade_count;
// } csm;

layout(row_major, std140, binding = 0) uniform Surface {
  mat4x3 model_matrix;
  vec4 base_color_tint;
} surface;

// layout(location = 2) uniform vec3 ambient_irradiance;
// layout(location = 3) uniform vec3 directional_light_irradiance;
// layout(location = 4) uniform vec3 directional_light_direction;

// int select_csm_cascade() {
//   float z = -vertex_data.view_space_position.z;
//   for (int i = 0; i < csm.cascade_count - 1; ++i) {
//     if (z < csm.cascades[i].far_plane_distance) {
//       return i;
//     }
//   }
//   return csm.cascade_count - 1;
// }

// float calculate_csm_shadow_factor(float n_dot_l) {
//   int i = select_csm_cascade();
//   vec3 p_world =
//     vertex_data.world_space_position +
//     0.5 * csm.cascades[i].pixel_length * tan(acos(n_dot_l)) * directional_light_direction;
//   vec3 p_clip = csm.cascades[i].view_clip_matrix * vec4(p_world, 1.0);
//   vec4 texcoord = vec4(p_clip.xy * vec2(0.5, -0.5) + 0.5, i, clamp(p_clip.z, 0.0, 1.0));
//   return texture(shadow_map, texcoord);
// }

void main() {
  vec3 color = texture(base_color_texture, vertex_data.texcoord).rgb * surface.base_color_tint.rgb;
  vec3 normal = normalize(vertex_data.world_space_normal);
  out_color = vec4(color, 1.0);
  out_normal = oct_encode(normal);
  // vec3 l = directional_light_direction;
  // float n_dot_l = dot(n, l);
  // float shadow_factor = calculate_csm_shadow_factor(n_dot_l);
  // vec3 irradiance = ambient_irradiance + directional_light_irradiance * max(n_dot_l, 0.0) * shadow_factor;
  // out_color = vec4(irradiance * base_color, 1.0);
}
)";

// constexpr auto wireframe_vertex_shader_source = R"(
// #version 460 core

// layout(location = 0) in vec3 model_space_position;

// layout(location = 0) uniform mat4 model_view_projection_matrix;

// void main() {
//   gl_Position = model_view_projection_matrix *
//   vec4(model_space_position, 1.0);
// }
// )";

// constexpr auto wireframe_fragment_shader_source = R"(
// #version 460 core

// layout(location = 0) out vec4 out_color;

// layout(location = 1) uniform vec3 in_color;

// void main() {
//   out_color = vec4(in_color, 1.0);
// }
// )";

auto constexpr fullscreen_vertex_shader_source = R"(
#version 460 core

out vec2 texcoord;

void main() {
  vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(-1.0, 3.0), vec2(3.0, -1.0));
  texcoord = positions[gl_VertexID] * vec2(0.5, -0.5) + 0.5;
  gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);
}
)";

auto constexpr lighting_fragment_shader_source = R"(
#version 460 core

in vec2 texcoord;

out vec4 out_color;

layout(binding = 0) uniform sampler2D depth_buffer;
layout(binding = 1) uniform sampler2D color_buffer;
layout(binding = 2) uniform sampler2D normal_buffer;
layout(binding = 3) uniform sampler2DArrayShadow shadow_map;

layout(row_major, std140, binding = 0) uniform Lighting {
  mat4x3 inverse_view_matrix;
  vec2 tan_half_fov;
  float near_plane_distance;
  vec4 ambient_irradiance;
  vec4 directional_light_irradiance;
  vec4 directional_light_direction;
};

vec3 decode_view_space_position(float d) {
  float z_view = -near_plane_distance / d;
  vec2 xy_ndc = vec2(texcoord.x * 2.0 - 1.0, texcoord.y * -2.0 + 1.0);
  vec2 xy_view = vec2(-1.0, 1.0) * xy_ndc * z_view * tan_half_fov;
  return vec3(xy_view, z_view);
}

vec3 decode_world_space_normal(vec2 e) {
  vec3 v = vec3(e.xy, 1.0 - abs(e.x) - abs(e.y));
  if (v.z < 0) {
    v.xy = (1.0 - abs(v.yx)) * (2.0 * step(0.0, v.xy) - 1.0);
  }
  return normalize(v);
}

struct Cascade {
  mat4x3 view_clip_matrix;
  float far_plane_distance;
  float pixel_length;
};

#define MAX_CASCADE_COUNT 4
layout(row_major, std140, binding = 1) uniform Cascaded_shadow_map {
  Cascade cascades[MAX_CASCADE_COUNT];
  int cascade_count;
} csm;

int select_csm_cascade(float z_view) {
  for (int i = 0; i < csm.cascade_count - 1; ++i) {
    if (-z_view < csm.cascades[i].far_plane_distance) {
      return i;
    }
  }
  return csm.cascade_count - 1;
}

float calculate_csm_shadow_factor(vec3 p_world, float z_view, float n_dot_l) {
  int i = select_csm_cascade(z_view);
  vec3 bias = 0.5 * csm.cascades[i].pixel_length * tan(acos(n_dot_l)) * directional_light_direction.xyz;
  vec3 p_clip = csm.cascades[i].view_clip_matrix * vec4(p_world + bias, 1.0);
  vec4 texcoord = vec4(p_clip.xy * vec2(0.5, -0.5) + 0.5, i, clamp(p_clip.z, 0.0, 1.0));
  return texture(shadow_map, texcoord);
}

void main() {
  float depth = texture(depth_buffer, texcoord).r;
  if (depth == 0.0) {
    discard;
  }
  vec3 p_view = decode_view_space_position(depth);
  vec3 p_world = inverse_view_matrix * vec4(p_view, 1.0);
  vec3 n_world = decode_world_space_normal(texture(normal_buffer, texcoord).rg);
  vec3 color = texture(color_buffer, texcoord).rgb;
  vec3 light = vec3(0.0);
  light += ambient_irradiance.rgb;
  float n_dot_l = dot(n_world, directional_light_direction.xyz);
  light +=
    directional_light_irradiance.rgb *
    max(n_dot_l, 0.0) *
    calculate_csm_shadow_factor(p_world + n_world * 0.01, p_view.z, n_dot_l);
  out_color = vec4(light * color, 1.0);
}
)";

auto constexpr temporal_antialiasing_fragment_shader_source = R"(
#version 460 core

in vec2 texcoord;

out vec4 out_color;

layout(binding = 0) uniform sampler2D accumulation_buffer;
layout(binding = 1) uniform sampler2D sample_buffer;

void main() {
  const float blend_factor = 1.0 / 32.0;
  vec3 accumulation_value = texture(accumulation_buffer, texcoord).rgb;
  vec3 sample_value = texture(sample_buffer, texcoord).rgb;
  out_color = vec4(mix(accumulation_value, sample_value, blend_factor), 1.0);
}
)";

auto constexpr postprocessing_fragment_shader_source = R"(
#version 460 core

in vec2 texcoord;

out vec4 out_color;

layout(binding = 0) uniform sampler2D in_color;

layout(location = 0) uniform float exposure;

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
  out_color = vec4(tonemap(exposure * texture(in_color, texcoord).rgb), 1.0);
}
)";

wrappers::Unique_texture make_default_base_color_texture() {
  auto result = wrappers::make_unique_texture(GL_TEXTURE_2D);
  auto const pixels = std::array<std::uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF};
  glTextureStorage2D(result.get(), 1, GL_RGBA8, 1, 1);
  glTextureSubImage2D(
      result.get(), 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  return result;
}

Mat4x4f perspective(Vec2f const &zoom,
                    float near_plane_distance,
                    Vec2f const &jitter = Vec2f::zero()) {
  return Mat4x4f{{zoom.x, 0.0f, jitter.x, 0.0f},
                 {0.0f, -zoom.y, jitter.y, 0.0f},
                 {0.0f, 0.0f, 0.0f, near_plane_distance},
                 {0.0f, 0.0f, -1.0f, 0.0f}};
}
} // namespace

Render_stream::Intrinsic_state::Intrinsic_state(
    Intrinsic_state_create_info const &)
    : _cascaded_shadow_map_intrinsic_state{{}},
      _surface_shader_program{wrappers::make_unique_shader_program(
          surface_vertex_shader_source, surface_fragment_shader_source)},
      _lighting_shader_program{wrappers::make_unique_shader_program(
          fullscreen_vertex_shader_source, lighting_fragment_shader_source)},
      _temporal_antialiasing_shader_program{
          wrappers::make_unique_shader_program(
              fullscreen_vertex_shader_source,
              temporal_antialiasing_fragment_shader_source)},
      _postprocessing_shader_program{wrappers::make_unique_shader_program(
          fullscreen_vertex_shader_source,
          postprocessing_fragment_shader_source)},
      _default_base_color_texture{make_default_base_color_texture()},
      _empty_vertex_array{wrappers::make_unique_vertex_array()} {}

Render_stream::Render_stream(
    Intrinsic_state const *intrinsic_state,
    Render_stream_create_info const &create_info) noexcept
    : _intrinsic_state{intrinsic_state},
      _target{static_cast<Render_target *>(create_info.target)},
      _scene{static_cast<Scene const *>(create_info.scene)},
      _camera{create_info.camera},
      _lighting_uniform_buffer{Uniform_buffer_create_info{.size = 112}} {}

void Render_stream::render() {
  update_surface_resource();
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(0.0f);
  glClipControl(GL_UPPER_LEFT, GL_ZERO_TO_ONE);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CW);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glEnable(GL_FRAMEBUFFER_SRGB);
  draw_cascaded_shadow_maps();
  auto const inverse_view_matrix =
      Mat4x4f::rigid(_camera->position, _camera->orientation);
  auto const view_matrix = rigid_inverse(inverse_view_matrix);
  draw_visibility_buffer(view_matrix);
  glDisable(GL_DEPTH_TEST);
  glBindVertexArray(_intrinsic_state->empty_vertex_array());
  do_lighting(inverse_view_matrix);
  do_temporal_antialiasing();
  do_postprocessing();
  _taa_resource.swap_accumulation_buffers();
}

void Render_stream::update_surface_resource() {
  if (_surface_resource.max_surfaces() != _scene->surfaces().max_size()) {
    _surface_resource = Surface_resource{{
        .max_surfaces = _scene->surfaces().max_size(),
    }};
  }
  _surface_resource.clear_mappings();
  for (auto const surface : _scene->surfaces()) {
    _surface_resource.add_mapping(surface);
  }
  _surface_resource.acquire();
  _surface_resource.update();
}

void Render_stream::draw_cascaded_shadow_maps() {
  // auto constexpr model_view_clip_matrix_location = 0;
  if (_camera->csm_cascade_count <= 0) {
    _cascaded_shadow_map = {};
  } else if (_cascaded_shadow_map.cascade_count() !=
                 _camera->csm_cascade_count ||
             _cascaded_shadow_map.texture_resolution() !=
                 _camera->csm_texture_resolution) {
    _cascaded_shadow_map = Cascaded_shadow_map{
        _intrinsic_state->cascaded_shadow_map_intrinsic_state(),
        {
            .texture_resolution = _camera->csm_texture_resolution,
            .cascade_count = _camera->csm_cascade_count,
        }};
  }
  if (_scene->directional_light() && _camera->csm_cascade_count > 0) {
    _cascaded_shadow_map.acquire();
    _cascaded_shadow_map.draw(*_scene,
                              *_camera,
                              _scene->directional_light()->direction,
                              _surface_resource);
  }
}

void Render_stream::draw_visibility_buffer(math::Mat4x4f const &view_matrix) {
  auto const target_extents = _target->get_extents();
  if (_visibility_buffer.extents() != target_extents) {
    _visibility_buffer = Visibility_buffer{{.extents = target_extents}};
  }
  auto const ndc_pixel_extents =
      Vec2f{2.0f / target_extents.x, 2.0f / target_extents.y};
  auto const jitter =
      Vec2f{(rand() / (float)RAND_MAX - 0.5f) * ndc_pixel_extents.x,
            (rand() / (float)RAND_MAX - 0.5f) * ndc_pixel_extents.y};
  auto const projection_matrix =
      perspective(_camera->zoom, _camera->near_plane_distance, jitter);
  // glBindFramebuffer(GL_FRAMEBUFFER, _target->get_framebuffer());
  glBindFramebuffer(GL_FRAMEBUFFER, _visibility_buffer.framebuffer());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glEnable(GL_POLYGON_OFFSET_FILL);
  // glPolygonOffset(1.0f, 1.0f);
  glViewport(0, 0, target_extents.x, target_extents.y);
  // glDisable(GL_BLEND);
  // glDisable(GL_DEPTH_CLAMP);
  draw_surfaces(projection_matrix * view_matrix);
  // glEnable(GL_POLYGON_OFFSET_LINE);
  // glPolygonOffset(1.0f, 1.0f);
  // glLineWidth(2.0f);
  // glDepthFunc(GL_GEQUAL);
  // // glEnable(GL_BLEND);
  // // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // // glEnable(GL_LINE_SMOOTH);
  // draw_wireframes(projection_matrix * view_matrix);
}

void Render_stream::draw_surfaces(Mat4x4f const &view_projection_matrix) {
  auto constexpr view_projection_matrix_location = 0;
  // auto constexpr exposure_location = 5;
  auto const shader_program = _intrinsic_state->surface_shader_program();
  // auto const ambient_irradiance = _scene->ambient_irradiance();
  // auto const &directional_light = _scene->directional_light();
  // auto const exposure = _camera->exposure;
  glUseProgram(shader_program);
  // glProgramUniform3f(shader_program,
  //                    ambient_irradiance_location,
  //                    ambient_irradiance.r,
  //                    ambient_irradiance.g,
  //                    ambient_irradiance.b);
  // if (directional_light) {
  //   if (_cascaded_shadow_map) {
  //     glBindTextureUnit(1, _cascaded_shadow_map.texture());
  //     glBindBufferBase(
  //         GL_UNIFORM_BUFFER, 0, _cascaded_shadow_map.uniform_buffer());
  //   }
  //   glProgramUniform3f(shader_program,
  //                      directional_light_irradiance_location,
  //                      directional_light->irradiance.r,
  //                      directional_light->irradiance.g,
  //                      directional_light->irradiance.b);
  //   glProgramUniform3f(shader_program,
  //                      directional_light_direction_location,
  //                      directional_light->direction.x,
  //                      directional_light->direction.y,
  //                      directional_light->direction.z);
  // } else {
  //   glProgramUniform3f(shader_program,
  //                      directional_light_irradiance_location,
  //                      0.0f,
  //                      0.0f,
  //                      0.0f);
  //   glProgramUniform3f(
  //       shader_program, directional_light_direction_location, 1.0f, 0.0f,
  //       0.0f);
  // }
  // glProgramUniform1f(shader_program, exposure_location, exposure);
  glProgramUniformMatrix4fv(shader_program,
                            view_projection_matrix_location,
                            1,
                            GL_TRUE,
                            &view_projection_matrix[0][0]);
  for (auto const surface : _scene->surfaces()) {
    if (!surface->visible) {
      continue;
    }
    // auto const model_matrix =
    //     Mat4x4f{surface->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
    // auto const model_view_matrix = view_matrix * model_matrix;
    // auto const model_view_clip_matrix = view_clip_matrix * model_matrix;
    // glProgramUniformMatrix4fv(
    //     shader_program, model_matrix_location, 1, GL_TRUE,
    //     &model_matrix[0][0]);
    auto const &material = surface->material;
    if (auto const base_color_texture =
            static_cast<Texture *>(material.base_color_texture)) {
      glBindTextureUnit(0, base_color_texture->get());
    } else {
      glBindTextureUnit(0, _intrinsic_state->default_base_color_texture());
    }
    glBindBufferRange(
        GL_UNIFORM_BUFFER,
        0,
        _surface_resource.uniform_buffer().get(),
        _surface_resource.get_mapping(surface).uniform_buffer_offset,
        64);
    auto const mesh = static_cast<Surface_mesh const *>(surface->mesh);
    mesh->bind_vertex_array();
    mesh->draw();
  }
  _surface_resource.release();
}

// void Render_stream::draw_wireframes(
//     math::Mat4x4f const &view_projection_matrix) {
//   auto constexpr model_view_projection_matrix_location = 0;
//   auto constexpr color_location = 1;
//   auto const shader_program = _intrinsic_state->wireframe_shader_program();
//   glUseProgram(shader_program);
//   for (auto const wireframe : _scene->wireframes()) {
//     if (!wireframe->visible) {
//       continue;
//     }
//     auto const model_matrix =
//         Mat4x4f{wireframe->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
//     auto const model_view_projection_matrix =
//         view_projection_matrix * model_matrix;
//     glProgramUniformMatrix4fv(shader_program,
//                               model_view_projection_matrix_location,
//                               1,
//                               GL_TRUE,
//                               &model_view_projection_matrix[0][0]);
//     glProgramUniform3f(shader_program,
//                        color_location,
//                        wireframe->color.r,
//                        wireframe->color.g,
//                        wireframe->color.b);
//     auto const mesh = static_cast<Wireframe_mesh const *>(wireframe->mesh);
//     mesh->bind_vertex_array();
//     mesh->draw();
//   }
// }

void Render_stream::do_lighting(Mat4x4f const &inverse_view_matrix) {
  _lighting_uniform_buffer.acquire();
  auto const data = _lighting_uniform_buffer.get().data();
  std::memcpy(data, &inverse_view_matrix, 48);
  auto const tan_half_fov =
      Vec2f{1.0f / _camera->zoom.x, 1.0f / _camera->zoom.y};
  std::memcpy(data + 48, &tan_half_fov, 8);
  std::memcpy(data + 56, &_camera->near_plane_distance, 4);
  auto const ambient_irradiance = _scene->ambient_irradiance();
  std::memcpy(data + 64, &ambient_irradiance, 12);
  if (auto const &directional_light = _scene->directional_light()) {
    std::memcpy(data + 80, &directional_light->irradiance, 12);
    std::memcpy(data + 96, &directional_light->direction, 12);
  } else {
    auto const v = Vec3f::zero();
    std::memcpy(data + 80, &v, 12);
    std::memcpy(data + 96, &v, 12);
  }
  auto const target_extents = _target->get_extents();
  if (_taa_resource.extents() != target_extents) {
    _taa_resource = Temporal_antialiasing_resource{{.extents = target_extents}};
  }
  auto const shader_program = _intrinsic_state->lighting_shader_program();
  glBindFramebuffer(GL_FRAMEBUFFER,
                    _taa_resource.sample_buffer().framebuffer());
  glClear(GL_COLOR_BUFFER_BIT);
  glUseProgram(shader_program);
  glBindTextureUnit(0, _visibility_buffer.depth_texture());
  glBindTextureUnit(1, _visibility_buffer.color_texture());
  glBindTextureUnit(2, _visibility_buffer.normal_texture());
  glBindTextureUnit(3, _cascaded_shadow_map.texture());
  glBindBufferBase(GL_UNIFORM_BUFFER, 0, _lighting_uniform_buffer.get().get());
  glBindBufferBase(GL_UNIFORM_BUFFER, 1, _cascaded_shadow_map.uniform_buffer());
  glDrawArrays(GL_TRIANGLES, 0, 3);
  _lighting_uniform_buffer.release();
  if (_cascaded_shadow_map) {
    _cascaded_shadow_map.release();
  }
}

void Render_stream::do_temporal_antialiasing() {
  glBindFramebuffer(GL_FRAMEBUFFER,
                    _taa_resource.curr_accumulation_buffer().framebuffer());
  glUseProgram(_intrinsic_state->temporal_antialiasing_shader_program());
  glBindTextureUnit(0, _taa_resource.prev_accumulation_buffer().texture());
  glBindTextureUnit(1, _taa_resource.sample_buffer().texture());
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

void Render_stream::do_postprocessing() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glUseProgram(_intrinsic_state->postprocessing_shader_program());
  glBindTextureUnit(0, _taa_resource.curr_accumulation_buffer().texture());
  glProgramUniform1f(
      _intrinsic_state->postprocessing_shader_program(), 0, _camera->exposure);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}
} // namespace gl
} // namespace graphics
} // namespace marlon