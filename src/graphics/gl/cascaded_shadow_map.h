#ifndef MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H
#define MARLON_GRAPHICS_GL_CASCADED_SHADOW_MAP_H

#include <util/list.h>

namespace marlon::graphics::gl {
auto constexpr max_cascaded_shadow_map_cascade_count = 8;

struct Cascaded_shadow_map_create_info {
  int cascade_count;
  int cascade_resolution;
};

class Cascaded_shadow_map {
public:
  class Cascade {};

  explicit Cascaded_shadow_map(
      Cascaded_shadow_map_create_info const &create_info);

private:
  util::Object_storage<Cascade[max_cascaded_shadow_map_cascade_count]>
      _cascade_storage;
};
} // namespace marlon::graphics::gl

#endif