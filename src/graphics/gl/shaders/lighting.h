#ifndef MARLON_GRAPHICS_GL_SHADERS_LIGHTING_H
#define MARLON_GRAPHICS_GL_SHADERS_LIGHTING_H

namespace marlon::graphics::gl {
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
  vec4 sky_irradiance;
  vec4 ground_albedo;
  vec4 directional_light_irradiance;
  vec4 directional_light_direction;
  uint frame_number;
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

uvec3 pcg3d(uvec3 v) {
  v = v * 1664525u + 1013904223u;

  v.x += v.y*v.z;
  v.y += v.z*v.x;
  v.z += v.x*v.y;

  v ^= v >> 16u;

  v.x += v.y*v.z;
  v.y += v.z*v.x;
  v.z += v.x*v.y;

  return v;
}

uvec3 hash(vec2 s) {
  uvec4 u = uvec4(s, uint(s.x) ^ uint(s.y), uint(s.x) + uint(s.y));
  return pcg3d(u.xyz);
}

vec3 rand(vec2 s) {
  return vec3(hash(s) * (1.0 / float(0xffffffffu)));
}

float calculate_csm_shadow_factor(vec3 p_world, float z_view, float n_dot_l) {
  int i = select_csm_cascade(z_view);
  vec2 random_offset = rand(gl_FragCoord.xy * frame_number).xy - 0.5;
  vec3 bias = (1.0 + length(random_offset)) * csm.cascades[i].pixel_length * tan(acos(n_dot_l)) * directional_light_direction.xyz;
  vec3 p_clip = csm.cascades[i].view_clip_matrix * vec4(p_world + bias, 1.0);
  vec4 texcoord = vec4(p_clip.xy * vec2(0.5, -0.5) + 0.5 + random_offset / textureSize(shadow_map, 0).xy, i, clamp(p_clip.z, 0.0, 1.0));
  return texture(shadow_map, texcoord);
}

#define PI 3.14159
#define INV_PI (1.0 / PI)
void main() {
  float depth = texture(depth_buffer, texcoord).r;
  if (depth == 0.0) {
    discard;
  }
  vec3 p_view = decode_view_space_position(depth);
  vec3 p_world = inverse_view_matrix * vec4(p_view, 1.0);
  vec3 color = texture(color_buffer, texcoord).rgb;
  vec3 n_world = decode_world_space_normal(texture(normal_buffer, texcoord).rg);
  vec3 irradiance = vec3(0.0);
  vec3 ground_irradiance =
    ground_albedo.rgb * (
      sky_irradiance.rgb +
      directional_light_irradiance.rgb * max(directional_light_direction.y, 0.0)
    );
  vec3 ambient_irradiance = mix(ground_irradiance, sky_irradiance.rgb, n_world.y * 0.5 + 0.5);
  irradiance += ambient_irradiance;
  float n_dot_l = dot(n_world, directional_light_direction.xyz);
  irradiance +=
    directional_light_irradiance.rgb * max(n_dot_l, 0.0) *
    calculate_csm_shadow_factor(p_world + n_world * 0.005, p_view.z, n_dot_l);
  out_color = vec4(color * INV_PI * irradiance, 1.0);
}
)";
}

#endif
