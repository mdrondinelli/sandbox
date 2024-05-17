#ifndef MARLON_GRAPHICS_GL_TEXTURE_H
#define MARLON_GRAPHICS_GL_TEXTURE_H

#include "../texture.h"
#include "wrappers/unique_texture.h"

namespace marlon {
namespace graphics {
namespace gl {
class Texture final : public graphics::Texture {
public:
  explicit Texture(Texture_create_info const &create_info);

  std::uint32_t get() const noexcept {
    return _handle.get();
  }

private:
  wrappers::Unique_texture _handle;
};
}
} // namespace graphics
} // namespace marlon

#endif