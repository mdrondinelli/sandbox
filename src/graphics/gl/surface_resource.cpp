#include "surface_resource.h"

#include <stdexcept>

namespace marlon::graphics::gl {
using namespace math;
using namespace util;

Surface_resource::Surface_resource(
    Surface_resource_create_info const &create_info)
    : _max_surfaces{create_info.max_surfaces},
      _uniform_buffers{
          Uniform_buffer_create_info{.size = align(64, 256) * _max_surfaces}} {
  _mappings.reserve(create_info.max_surfaces);
}

util::Size Surface_resource::max_surfaces() const noexcept {
  return _max_surfaces;
}

Surface_resource::Mapping const &
Surface_resource::get_mapping(Surface const *surface) const noexcept {
  return _mappings.find(surface)->second;
}

Surface_resource::Mapping const &
Surface_resource::add_mapping(Surface const *surface) {
  if (_mappings.size() < _max_surfaces) {
    return _mappings
        .emplace(surface,
                 Mapping{
                     .uniform_buffer_offset = 256 * _mappings.size(),
                 })
        .first->second;
  } else {
    throw std::runtime_error{"Surface_resource max_surfaces exceeded"};
  }
}

void Surface_resource::clear_mappings() noexcept { _mappings.clear(); }

void Surface_resource::acquire() { _uniform_buffers.acquire(); }

void Surface_resource::prepare(Mat4x4f const &view_projection_matrix) {
  auto const uniform_buffer_base = _uniform_buffers.get().data();
  for (auto const &[surface, mapping] : _mappings) {
    auto const surface_base =
        uniform_buffer_base + mapping.uniform_buffer_offset;
    auto const &model_matrix = surface->transform;
    auto const current_model_view_projection_matrix =
        view_projection_matrix *
        Mat4x4f{model_matrix, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const previous_model_view_projection_matrix =
        _previous_view_projection_matrix
            ? *_previous_view_projection_matrix *
                  Mat4x4f{model_matrix, {0.0f, 0.0f, 0.0f, 1.0f}}
            : current_model_view_projection_matrix;
    auto const base_color_tint = Vec4f{surface->material.base_color_tint.r,
                                       surface->material.base_color_tint.g,
                                       surface->material.base_color_tint.b,
                                       1.0f};
    std::memcpy(surface_base, &current_model_view_projection_matrix, 64);
    std::memcpy(surface_base + 64, &previous_model_view_projection_matrix, 64);
    std::memcpy(surface_base + 128, &model_matrix, 48);
    std::memcpy(surface_base + 176, &base_color_tint, 16);
  }
  _previous_view_projection_matrix = view_projection_matrix;
}

void Surface_resource::release() { _uniform_buffers.release(); }
} // namespace marlon::graphics::gl