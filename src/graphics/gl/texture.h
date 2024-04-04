#ifndef MARLON_GRAPHICS_GL_TEXTURE_H
#define MARLON_GRAPHICS_GL_TEXTURE_H

#include "../texture.h"
#include "unique_texture_handle.h"

namespace marlon {
namespace graphics {
class Gl_texture final : public Texture {
  friend class Gl_scene;

public:
  explicit Gl_texture(Texture_create_info const &create_info);

private:
  Gl_unique_texture_handle _handle;
};
} // namespace graphics
} // namespace marlon

#endif