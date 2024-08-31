#ifndef MARLON_GRAPHICS_GL_SHADERS_FULLSCREEN_H
#define MARLON_GRAPHICS_GL_SHADERS_FULLSCREEN_H

namespace marlon::graphics::gl {
auto constexpr fullscreen_vertex_shader_source = R"(
#version 460 core

out vec2 texcoord;

void main() {
  vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(-1.0, 3.0), vec2(3.0, -1.0));
  texcoord = positions[gl_VertexID] * vec2(0.5, -0.5) + 0.5;
  gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);
}
)";
}

#endif
