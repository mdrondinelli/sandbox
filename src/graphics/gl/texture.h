#ifndef MARLON_GRAPHICS_GL_TEXTURE_H
#define MARLON_GRAPHICS_GL_TEXTURE_H

#include "../texture.h"
#include "scene.h"
#include "unique_texture_handle.h"

namespace marlon {
namespace graphics {
namespace gl {
class Texture final : public graphics::Texture {
  friend class ::marlon::graphics::gl::Scene;

public:
  explicit Texture(Texture_create_info const &create_info);

private:
  Unique_texture_handle _handle;
};
}
} // namespace graphics
} // namespace marlon

#endif