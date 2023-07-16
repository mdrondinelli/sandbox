#include "render_stream.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_render_stream::Impl::Impl(
    Render_stream_create_info const &create_info) noexcept
    : _source_scene{static_cast<Gl_scene *>(create_info.source_scene)},
      _source_camera_instance{static_cast<Gl_camera_instance *>(
          create_info.source_camera_instance)},
      _destination{
          static_cast<Gl_render_destination *>(create_info.destination)} {}

void Gl_render_stream::Impl::render(
    std::uint32_t shader_program,
    std::int32_t model_view_matrix_uniform_location) {
  glBindFramebuffer(GL_FRAMEBUFFER, _destination->get_framebuffer());
  glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(shader_program);
  _source_scene->_impl.draw_surface_instances(
      shader_program, model_view_matrix_uniform_location);
}

Gl_render_stream::Gl_render_stream(
    Render_stream_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon