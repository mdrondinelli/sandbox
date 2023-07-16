#ifndef MARLON_RENDERING_GL_RENDER_STREAM_H
#define MARLON_RENDERING_GL_RENDER_STREAM_H

#include "../render_stream.h"
#include "camera_instance.h"
#include "render_destination.h"
#include "scene.h"

namespace marlon {
namespace rendering {
class Gl_render_stream : public Render_stream {
  friend class Gl_render_engine;
public:
  class Impl {
  public:
    explicit Impl(Render_stream_create_info const &create_info) noexcept;

    void render(std::uint32_t shader_program);

  private:
    Gl_scene *_source_scene;
    Gl_camera_instance *_source_camera_instance;
    Gl_render_destination *_destination;
  };

  explicit Gl_render_stream(
      Render_stream_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif