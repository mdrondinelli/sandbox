#include "render_group.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_render_group::Gl_render_group(Render_group_create_info const &create_info) {
  for (auto const &render_info : create_info.render_infos) {
    _render_infos.push_back(
        {static_cast<Gl_camera_instance *>(render_info.source),
         static_cast<Gl_render_destination *>(render_info.destination)});
  }
}

void Gl_render_group::render() {
  for (auto const &render_info : _render_infos) {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                      render_info.destination->framebuffer());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(_shader_program);
  }
}
} // namespace rendering
} // namespace marlon