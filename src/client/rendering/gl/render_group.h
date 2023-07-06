#ifndef MARLON_RENDERING_GL_RENDER_GROUP_H
#define MARLON_RENDERING_GL_RENDER_GROUP_H

#include <vector>

#include "../render_group.h"
#include "camera_instance.h"
#include "render_destination.h"

namespace marlon {
namespace rendering {
struct Gl_render_info {
  Gl_camera_instance *source;
  Gl_render_destination *destination;
};

class Gl_render_group : public Render_group {
public:
  explicit Gl_render_group(
      Render_group_create_info const &create_info) noexcept;

  void render() final;

private:
  std::vector<Gl_render_info> _render_infos;
  std::uint32_t _shader_program;
};
} // namespace rendering
} // namespace marlon

#endif