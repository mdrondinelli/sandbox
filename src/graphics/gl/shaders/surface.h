#ifndef MARLON_GRAPHICS_GL_SHADERS_SURFACE_H
#define MARLON_GRAPHICS_GL_SHADERS_SURFACE_H

namespace marlon::graphics::gl {
constexpr auto surface_vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;
layout(location = 1) in vec3 model_space_normal;
layout(location = 2) in vec2 texcoord;

out Vertex_data {
  vec3 model_space_position;
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) uniform mat4 jitter_matrix;

layout(row_major, std140, binding = 0) uniform Surface {
  mat4 current_model_view_projection_matrix;
  mat4 previous_model_view_projection_matrix;
  mat4x3 model_matrix;
  vec4 base_color_tint;
} surface;

void main() {
  vertex_data.model_space_position = model_space_position;
  vertex_data.world_space_normal = mat3(surface.model_matrix) * model_space_normal;
  vertex_data.texcoord = texcoord;
  gl_Position = jitter_matrix * (surface.current_model_view_projection_matrix * vec4(model_space_position, 1.0));
}
)";

constexpr auto surface_fragment_shader_source = R"(
#version 460 core

#define MAX_CASCADE_COUNT 4

in Vertex_data {
  vec3 model_space_position;
  vec3 world_space_normal;
  vec2 texcoord;
} vertex_data;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec2 out_normal;
layout(location = 2) out vec2 out_motion_vector;

vec2 oct_encode(vec3 v) {
  vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + abs(v.z)));
  return v.z <= 0.0 ? (1.0 - abs(p.yx)) * (2.0 * step(0.0, p) - 1.0) : p;
}

layout(row_major, std140, binding = 0) uniform Surface {
  mat4 current_model_view_projection_matrix;
  mat4 previous_model_view_projection_matrix;
  mat4x3 model_matrix;
  vec4 base_color_tint;
} surface;

void main() {
  vec3 color = surface.base_color_tint.rgb;
  vec3 normal = normalize(vertex_data.world_space_normal);
  out_color = vec4(color, 1.0);
  out_normal = oct_encode(normal);
  vec4 current_clip_space_position = surface.current_model_view_projection_matrix * vec4(vertex_data.model_space_position, 1.0);
  vec4 previous_clip_space_position = surface.previous_model_view_projection_matrix * vec4(vertex_data.model_space_position, 1.0);
  vec2 current_ndc_position = current_clip_space_position.xy / current_clip_space_position.w;
  vec2 previous_ndc_position = previous_clip_space_position.xy / previous_clip_space_position.w;
  vec2 current_texcoord = vec2(0.5, -0.5) * current_ndc_position + 0.5;
  vec2 previous_texcoord = vec2(0.5, -0.5) * previous_ndc_position + 0.5;
  out_motion_vector = previous_texcoord - current_texcoord;
}
)";
} // namespace marlon::graphics::gl

#endif
