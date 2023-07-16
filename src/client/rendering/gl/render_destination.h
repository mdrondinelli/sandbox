#ifndef MARLON_RENDERING_GL_RENDER_DESTINATION_H
#define MARLON_RENDERING_GL_RENDER_DESTINATION_H

#include <cstdint>

#include "../render_destination.h"

namespace marlon {
namespace rendering {
class Gl_render_destination : public Render_destination {
public:
  virtual ~Gl_render_destination() = default;

  virtual std::uint32_t get_framebuffer() const noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif