#ifndef MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H
#define MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H

#include <util/list.h>

namespace marlon::graphics::gl {
struct Cascaded_shadow_map_create_info {
  int count;
  int resolution;
};

class Cascaded_shadow_map {
public:
  class Cascade {};

  explicit Cascaded_shadow_map(
      Cascaded_shadow_map_create_info const &create_info);

private:
};
} // namespace marlon::graphics::gl

#endif