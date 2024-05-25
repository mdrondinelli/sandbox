#ifndef MARLON_GRAPHICS_GL_SURFACE_RESOURCE_H
#define MARLON_GRAPHICS_GL_SURFACE_RESOURCE_H

#include <optional>

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
  struct Surface_data {
    util::Size uniform_buffer_offset;
    math::Mat4x4f model_view_projection_matrix;
    bool marked;
  };

public:
  Surface_resource() noexcept = default;

  Surface_resource(Surface_resource_create_info const &create_info);

  util::Size max_surfaces() const noexcept;

  // only call when acquired
  std::uint32_t uniform_buffer() const noexcept;

  // only call when acquired
  util::Size uniform_buffer_offset(Surface const *surface) const noexcept;

  // only call when acquired
  void write(Surface const *surface,
             math::Mat4x4f const &view_projection_matrix);

  void acquire();

  void release();

  // void prepare(math::Mat4x4f const &view_projection_matrix);

private:
  util::Allocating_map<Surface const *, Surface_data> _surfaces;
  Triple_buffer<Uniform_buffer> _uniform_buffers;
  util::Size _uniform_buffer_offset{};
};
} // namespace marlon::graphics::gl

#endif