#ifndef MARLON_GRAPHICS_GL_TEMPORAL_ANTIALIASING_RESOURCE_H
#define MARLON_GRAPHICS_GL_TEMPORAL_ANTIALIASING_RESOURCE_H

#include <array>

#include <math/vec.h>

#include "temporal_accumulation_buffer.h"

namespace marlon::graphics::gl {
struct Temporal_antialiasing_resource_create_info {
  math::Vec2i extents;
};

class Temporal_antialiasing_resource {
public:
  Temporal_antialiasing_resource() noexcept = default;

  explicit Temporal_antialiasing_resource(
      Temporal_antialiasing_resource_create_info const &create_info);

  operator bool() const noexcept { return extents() != math::Vec2i::zero(); }

  math::Vec2i const &extents() const noexcept { return _extents; }

  Temporal_accumulation_buffer const &
  prev_accumulation_buffer() const noexcept {
    return _accumulation_buffers[_accumulation_buffer_index];
  }

  Temporal_accumulation_buffer const &
  curr_accumulation_buffer() const noexcept {
    return _accumulation_buffers[1 - _accumulation_buffer_index];
  }

  Temporal_accumulation_buffer const &sample_buffer() const noexcept {
    return _sample_buffer;
  }

  void swap_accumulation_buffers() noexcept {
    _accumulation_buffer_index = 1 - _accumulation_buffer_index;
  }

private:
  math::Vec2i _extents{};
  std::array<Temporal_accumulation_buffer, 2> _accumulation_buffers;
  Temporal_accumulation_buffer _sample_buffer;
  int _accumulation_buffer_index{};
};
} // namespace marlon::graphics::gl

#endif