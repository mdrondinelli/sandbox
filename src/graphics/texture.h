#ifndef MARLON_GRAPHICS_TEXTURE_H
#define MARLON_GRAPHICS_TEXTURE_H

#include <util/memory.h>

namespace marlon::graphics {
enum class Texture_color_space { linear, srgb };

enum class Texture_format { rgba8 };

struct Texture_create_info {
  Texture_color_space color_space;
  Texture_format format;
  util::Block data;
};

class Texture {};
} // namespace marlon

#endif
