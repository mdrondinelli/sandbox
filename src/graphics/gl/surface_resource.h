#ifndef MARLON_GRAPHICS_GL_SURFACE_RESOURCE_H
#define MARLON_GRAPHICS_GL_SURFACE_RESOURCE_H

#include <util/map.h>
#include <util/size.h>

#include "../surface.h"
#include "multiple_buffer.h"
#include "uniform_buffer.h"

namespace marlon::graphics::gl {
struct Surface_resource_create_info {
  util::Size max_surfaces;
};

class Surface_resource {
public:
  struct Mapping {
    util::Size uniform_buffer_offset;
  };

  Surface_resource() noexcept = default;

  Surface_resource(Surface_resource_create_info const &create_info);

  util::Size max_surfaces() const noexcept;

  Mapping const &get_mapping(Surface const *surface) const noexcept;

  Mapping const &add_mapping(Surface const *surface);

  void clear_mappings() noexcept;

  Uniform_buffer const &uniform_buffer() const noexcept {
    return _uniform_buffers.get();
  }

  void acquire();

  void update();

  void release();

private:
  util::Size _max_surfaces{};
  util::Allocating_map<Surface const *, Mapping> _mappings;
  Triple_buffer<Uniform_buffer> _uniform_buffers;
};
} // namespace marlon::graphics::gl

#endif