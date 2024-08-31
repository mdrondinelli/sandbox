#ifndef MARLON_GRAPHICS_GL_SHADERS_TEMPORAL_ANTIALIASING_H
#define MARLON_GRAPHICS_GL_SHADERS_TEMPORAL_ANTIALIASING_H

namespace marlon::graphics::gl {
auto constexpr temporal_antialiasing_fragment_shader_source = R"(
#version 460 core

in vec2 texcoord;

out vec4 out_color;

layout(binding = 0) uniform sampler2D accumulation_buffer;
layout(binding = 1) uniform sampler2D depth_buffer;
layout(binding = 2) uniform sampler2D motion_vector_buffer;
layout(binding = 3) uniform sampler2D lighting_buffer;

layout(location = 0) uniform float ideal_blend_factor;

vec2 get_motion_vector() {
  vec2 pixel_size = vec2(1.0) / textureSize(depth_buffer, 0);
  float nearest_depth = 0.0;
  vec2 nearest_neighbor_uv = texcoord;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      vec2 uv = texcoord + vec2(x, y) * pixel_size;
      float depth = texture(depth_buffer, uv).r;
      if (depth > nearest_depth) {
        nearest_depth = depth;
        nearest_neighbor_uv = uv;
      }
    }
  }
  return texture(motion_vector_buffer, nearest_neighbor_uv).rg;
}

void main() {
  vec2 motion_vector = get_motion_vector();
  vec2 accumulation_texcoord = texcoord + vec2(1.0, 1.0) * motion_vector;
  float blend_factor = 1.0;
  if (accumulation_texcoord == clamp(accumulation_texcoord, vec2(0.0), vec2(1.0))) {
    blend_factor = ideal_blend_factor;
  }
  vec3 sample_value = max(texture(lighting_buffer, texcoord).rgb, vec3(0.0));
  vec3 neighborhood_min = sample_value;
  vec3 neighborhood_max = sample_value;
  vec2[8] neighborhood_sample_offsets = vec2[8](
    vec2(-1, -1), vec2(-1, 0), vec2(-1, 1), vec2(0, -1), vec2(0, 1), vec2(1, -1), vec2(1, 0), vec2(1, 1)
  );
  for (int i = 0; i < 8; ++i) {
    vec2 neighbor_texcoord = texcoord + neighborhood_sample_offsets[i] / textureSize(lighting_buffer, 0);
    vec3 neighbor_value = max(texture(lighting_buffer, neighbor_texcoord).rgb, vec3(0.0));
    neighborhood_min = min(neighborhood_min, neighbor_value);
    neighborhood_max = max(neighborhood_max, neighbor_value);
  }
  vec3 accumulation_value = texture(accumulation_buffer, accumulation_texcoord).rgb;
  vec3 clamped_accumulation_value = clamp(accumulation_value, neighborhood_min, neighborhood_max);
  out_color = vec4(mix(clamped_accumulation_value, sample_value, blend_factor), 1.0);
}
)";
}

#endif
