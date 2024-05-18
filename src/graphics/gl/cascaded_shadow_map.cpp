#include "cascaded_shadow_map.h"

#include <iostream>
#include <stdexcept>

#include <glad/gl.h>

namespace marlon::graphics::gl {
using namespace math;

Cascaded_shadow_map::Cascade::Cascade(std::uint32_t texture, std::int32_t layer)
    : _framebuffer{wrappers::make_unique_framebuffer()} {
  glNamedFramebufferTextureLayer(
      _framebuffer.get(), GL_DEPTH_ATTACHMENT, texture, 0, layer);
  if (glCheckNamedFramebufferStatus(_framebuffer.get(), GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error{
        "Failed to make a complete framebuffer for CSM cascade"};
  }
}

Cascaded_shadow_map::Cascaded_shadow_map(
    Cascaded_shadow_map_create_info const &create_info)
    : _texture_resolution{create_info.texture_resolution},
      _texture{wrappers::make_unique_texture(GL_TEXTURE_2D_ARRAY)},
      _uniform_buffers{
          Uniform_buffer_create_info{.size = 64 * max_cascade_count + 4}} {
  glTextureStorage3D(_texture.get(),
                     1,
                     GL_DEPTH_COMPONENT32F,
                     _texture_resolution,
                     _texture_resolution,
                     create_info.cascade_count);
  glTextureParameteri(
      _texture.get(), GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
  glTextureParameteri(_texture.get(), GL_TEXTURE_COMPARE_FUNC, GL_GEQUAL);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTextureParameteri(_texture.get(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTextureParameteri(_texture.get(), GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTextureParameteri(_texture.get(), GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
  for (auto i = 0; i != create_info.cascade_count; ++i) {
    _cascades.emplace_back(_texture.get(), i);
  }
}

void Cascaded_shadow_map::update_frusta(Vec3f const &camera_position,
                                        Vec3f const &camera_direction,
                                        Vec2f const &camera_zoom,
                                        float camera_near_plane_distance,
                                        float camera_csm_render_distance,
                                        int camera_csm_cascade_count,
                                        Vec3f const &light_z_axis) {
  auto constexpr lambda = 0.9f;
  auto const c_log = [&](float i) {
    return camera_near_plane_distance *
           pow(camera_csm_render_distance / camera_near_plane_distance,
               i / camera_csm_cascade_count);
  };
  auto const c_uni = [&](float i) {
    return camera_near_plane_distance +
           (camera_csm_render_distance - camera_near_plane_distance) *
               (i / camera_csm_cascade_count);
  };
  auto const tan_squared_alpha = length_squared(camera_zoom);
  auto const tan_alpha = sqrt(tan_squared_alpha);
  auto const temp_y_axis = abs(light_z_axis.x) < abs(light_z_axis.y)
                               ? Vec3f::x_axis()
                               : Vec3f::y_axis();
  auto const light_x_axis = normalize(cross(temp_y_axis, light_z_axis));
  auto const light_y_axis = cross(light_z_axis, light_x_axis);
  for (auto i = 0; i < cascade_count(); ++i) {
    auto const cascade_near_log = c_log(i);
    auto const cascade_near_uni = c_uni(i);
    auto const cascade_far_log = c_log(i + 1);
    auto const cascade_far_uni = c_uni(i + 1);
    auto const cascade_near =
        lambda * cascade_near_log + (1.0f - lambda) * cascade_near_uni;
    auto const cascade_far =
        lambda * cascade_far_log + (1.0f - lambda) * cascade_far_uni;
    auto const sphere_distance =
        min(0.5f * (cascade_near + cascade_far) * (1.0f + tan_squared_alpha),
            cascade_far);
    auto const sphere_radius =
        length(Vec2f{cascade_far * tan_alpha, cascade_far - sphere_distance});
    auto const sphere_center_world_space =
        camera_position + camera_direction * sphere_distance;
    auto const sphere_center_light_space =
        Vec3f{dot(sphere_center_world_space, light_x_axis),
              dot(sphere_center_world_space, light_y_axis),
              dot(sphere_center_world_space, light_z_axis)};
    auto &cascade = cascades()[i];
    cascade.view_clip_matrix(
        Mat3x4f::orthographic(sphere_center_light_space.x - sphere_radius,
                              sphere_center_light_space.x + sphere_radius,
                              sphere_center_light_space.y - sphere_radius,
                              sphere_center_light_space.y + sphere_radius,
                              sphere_center_light_space.z + sphere_radius,
                              sphere_center_light_space.z - sphere_radius) *
        Mat4x4f{{light_x_axis.x, light_x_axis.y, light_x_axis.z, 0.0f},
                {light_y_axis.x, light_y_axis.y, light_y_axis.z, 0.0f},
                {light_z_axis.x, light_z_axis.y, light_z_axis.z, 0.0f},
                {0.0f, 0.0f, 0.0f, 1.0f}});
    cascade.render_distance(cascade_far);
    cascade.pixel_length(2.0f * sphere_radius / _texture_resolution);
  }
}

void Cascaded_shadow_map::acquire_uniform_buffer() {
  _uniform_buffers.acquire();
}

void Cascaded_shadow_map::update_uniform_buffer() {
  auto const data = _uniform_buffers.get().data();
  for (auto i = 0; i < cascade_count(); ++i) {
    auto const &cascade = cascades()[i];
    auto const &view_clip_matrix = cascade.view_clip_matrix();
    auto const render_distance = cascade.render_distance();
    auto const pixel_length = cascade.pixel_length();
    std::memcpy(data + 64 * i + 0, &view_clip_matrix, 48);
    std::memcpy(data + 64 * i + 48, &render_distance, 4);
    std::memcpy(data + 64 * i + 52, &pixel_length, 4);
  }
  auto const count = static_cast<int32_t>(cascade_count());
  std::memcpy(data + 64 * max_cascade_count, &count, 4);
}

void Cascaded_shadow_map::release_uniform_buffer() {
  _uniform_buffers.release();
}
} // namespace marlon::graphics::gl