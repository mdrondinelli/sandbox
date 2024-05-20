#include "temporal_antialiasing_resource.h"

namespace marlon::graphics::gl {
Temporal_antialiasing_resource::Temporal_antialiasing_resource(
    Temporal_antialiasing_resource_create_info const &create_info)
    : _extents{create_info.extents},
      _accumulation_buffers{
          Temporal_accumulation_buffer{{.extents = create_info.extents}},
          Temporal_accumulation_buffer{{.extents = create_info.extents}},
      },
      _sample_buffer{{.extents = create_info.extents}} {}
} // namespace marlon::graphics::gl