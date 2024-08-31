#ifndef MARLON_GRAPHICS_GL_SHADERS_POSTPROCESSING_H
#define MARLON_GRAPHICS_GL_SHADERS_POSTPROCESSING_H

namespace marlon::graphics::gl {
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
  out_color = vec4(tonemap(8 * exposure * texture(in_color, texcoord).rgb), 1.0);
}
)";
}

#endif
