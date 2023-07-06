#ifndef MARLON_RENDERING_GL_DEFAULT_RENDER_DESTINATION
#define MARLON_RENDERING_GL_DEFAULT_RENDER_DESTINATION

#include "render_destination.h"

namespace marlon {
namespace rendering {
class Gl_default_render_destination : public Gl_render_destination {
public:
  std::uint32_t framebuffer() const noexcept final;
};
} // namespace rendering
} // namespace marlon

#endif