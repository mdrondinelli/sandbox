#include "surface_resource.h"

#include <cstring>

#include <stdexcept>

namespace marlon::graphics::gl {
using namespace math;
using namespace util;

using Suballocator = Stack_allocator<>;

namespace {
auto constexpr uniform_buffer_stride = 256;
}
Surface_resource::Surface_resource(Surface_resource_create_info const &create_info)
    : _uniform_buffers{Uniform_buffer_create_info{
          .size = uniform_buffer_stride * create_info.max_surfaces,
      }} {
  _surfaces.reserve(create_info.max_surfaces);
}

util::Size Surface_resource::max_surfaces() const noexcept { return _surfaces.max_size() / 2; }

std::uint32_t Surface_resource::uniform_buffer() const noexcept { return _uniform_buffers.get().get(); }

util::Size Surface_resource::uniform_buffer_offset(Surface const *surface) const noexcept {
  return _surfaces.at(surface).uniform_buffer_offset;
}

void Surface_resource::acquire() {
  _uniform_buffers.acquire();
  _uniform_buffer_offset = 0;
  auto it = _surfaces.begin();
  auto const end = _surfaces.end();
  while (it != end) {
    if (it->second.marked) {
      it->second.marked = false;
      ++it;
    } else {
      it = _surfaces.erase(it);
    }
  }
}

void Surface_resource::release() { _uniform_buffers.release(); }

void Surface_resource::write(Surface const *surface, Mat4x4f const &view_projection_matrix) {
  auto const &model_matrix = surface->transform;
  auto const model_view_projection_matrix = view_projection_matrix * Mat4x4f{model_matrix, {0.0f, 0.0f, 0.0f, 1.0f}};
  auto it = _surfaces.find(surface);
  if (it == _surfaces.end()) {
    it = _surfaces
             .emplace(surface,
                      Surface_data{.uniform_buffer_offset = 0,
                                   .model_view_projection_matrix = model_view_projection_matrix,
                                   .marked = false})
             .first;
  }
  auto const data = _uniform_buffers.get().data() + _uniform_buffer_offset;
  std::memcpy(data, &model_view_projection_matrix, 64);
  std::memcpy(data + 64, &it->second.model_view_projection_matrix, 64);
  std::memcpy(data + 128, &model_matrix, 48);
  std::memcpy(data + 176, &surface->material.base_color_tint, 16);
  it->second.uniform_buffer_offset = _uniform_buffer_offset;
  it->second.model_view_projection_matrix = model_view_projection_matrix;
  it->second.marked = true;
  _uniform_buffer_offset += uniform_buffer_stride;
}
} // namespace marlon::graphics::gl
