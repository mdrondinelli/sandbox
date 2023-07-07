#include "render.h"

namespace marlon {
namespace rendering {
Gl_render::Gl_render(Render_create_info const &create_info) noexcept
    : _source{static_cast<Gl_camera_instance *>(create_info.source)},
      _destination{
          static_cast<Gl_render_destination *>(create_info.destination)} {}

Gl_camera_instance *Gl_render::source() const noexcept { return _source; }

Gl_render_destination *Gl_render::destination() const noexcept {
  return _destination;
}
} // namespace rendering
} // namespace marlon