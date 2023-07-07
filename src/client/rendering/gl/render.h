#ifndef MARLON_RENDERING_GL_RENDER_H
#define MARLON_RENDERING_GL_RENDER_H

#include "../render.h"
#include "camera_instance.h"
#include "render_destination.h"

namespace marlon {
namespace rendering {
class Gl_render : public Render {
public:
  explicit Gl_render(Render_create_info const &create_info) noexcept;

  Gl_camera_instance *source() const noexcept final;

  Gl_render_destination *destination() const noexcept final;

private:
  Gl_camera_instance *_source;
  Gl_render_destination *_destination;
};
} // namespace rendering
} // namespace marlon

#endif