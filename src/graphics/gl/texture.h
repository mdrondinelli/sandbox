#ifndef MARLON_GRAPHICS_GL_TEXTURE_H
#define MARLON_GRAPHICS_GL_TEXTURE_H

#include "../texture.h"
#include "unique_texture_handle.h"

namespace marlon {
namespace graphics {
class Gl_texture : public Texture {
public:
  explicit Gl_texture(Texture_create_info const &create_info);

private:
};
} // namespace graphics
} // namespace marlon

#endif