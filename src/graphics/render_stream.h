#ifndef MARLON_GRAPHICS_RENDER_STREAM_H
#define MARLON_GRAPHICS_RENDER_STREAM_H

#include "camera.h"
#include "render_target.h"
#include "scene.h"

namespace marlon {
namespace graphics {
struct Render_stream_create_info {
  Render_target *target;
  Scene const *scene;
  Camera const *camera;
};

class Render_stream {
public:
  virtual void render() = 0;

  virtual Render_target *target() const noexcept = 0;

  virtual Scene const *scene() const noexcept = 0;

  virtual Camera const *camera() const noexcept = 0;
};
} // namespace graphics
} // namespace marlon

#endif