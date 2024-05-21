#include "cascaded_shadow_map.h"

#include <iostream>
#include <stdexcept>

#include <glad/gl.h>

#include "surface_mesh.h"

namespace marlon::graphics::gl {
using namespace math;
namespace {
auto constexpr vertex_shader_source = R"(
#version 460 core

layout(location = 0) in vec3 model_space_position;

layout(row_major, std140, binding = 0) uniform Cascade {
  mat4x3 view_clip_matrix;
};

layout(row_major, std140, binding = 1) uniform Surface {
  mat4 current_model_view_projection_matrix;
  mat4 previous_model_view_projection_matrix;
  mat4x3 model_matrix;
  vec4 base_color_tint;
};

void main() {
  gl_Position = vec4(view_clip_matrix * vec4(model_matrix * vec4(model_space_position, 1.0), 1.0), 1.0);
}
)";

auto constexpr fragment_shader_source = R"(
#version 460 core

void main() {
}
)";
} // namespace

Cascaded_shadow_map::Intrinsic_state::Intrinsic_state(
    Intrinsic_state_create_info const &)
    : _shader_program{wrappers::make_unique_shader_program(
          vertex_shader_source, fragment_shader_source)} {}

Cascaded_shadow_map::Cascade::Cascade(std::uint32_t texture, std::int32_t layer)
    : _framebuffer{wrappers::make_unique_framebuffer()},
      _uniform_buffers{Uniform_buffer_create_info{.size = 64}} {
  glNamedFramebufferTextureLayer(
      _framebuffer.get(), GL_DEPTH_ATTACHMENT, texture, 0, layer);
  if (glCheckNamedFramebufferStatus(_framebuffer.get(), GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error{
        "Failed to make a complete framebuffer for CSM cascade"};
  }
}

Cascaded_shadow_map::Cascaded_shadow_map(
    Intrinsic_state const *intrinsic_state,
    Cascaded_shadow_map_create_info const &create_info)
    : _intrinsic_state{intrinsic_state},
      _texture_resolution{create_info.texture_resolution},
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

void Cascaded_shadow_map::acquire() {
  for (auto &cascade : cascades()) {
    cascade.acquire();
  }
  _uniform_buffers.acquire();
}

void Cascaded_shadow_map::release() {
  for (auto &cascade : cascades()) {
    cascade.release();
  }
  _uniform_buffers.release();
}

void Cascaded_shadow_map::draw(Scene const &scene,
                               Camera const &camera,
                               Vec3f const &light_z_axis,
                               Surface_resource const &surface_resource) {
  auto constexpr lambda = 3.0f / 4.0f;
  auto const c_log = [&](float i) {
    return camera.near_plane_distance *
           pow(camera.csm_render_distance / camera.near_plane_distance,
               i / camera.csm_cascade_count);
  };
  auto const c_uni = [&](float i) {
    return camera.near_plane_distance +
           (camera.csm_render_distance - camera.near_plane_distance) *
               (i / camera.csm_cascade_count);
  };
  auto const camera_to_world_matrix =
      Mat3x4f::rigid(camera.position, camera.orientation);
  auto const tan_half_fov = Vec2f{1.0f / camera.zoom.x, 1.0f / camera.zoom.y};
  auto const temp_axis = abs(light_z_axis.x) < abs(light_z_axis.y)
                             ? Vec3f::x_axis()
                             : Vec3f::y_axis();
  auto const light_x_axis = normalize(cross(temp_axis, light_z_axis));
  auto const light_y_axis = cross(light_z_axis, light_x_axis);
  auto const world_to_light_matrix = Mat3x4f{
      {light_x_axis.x, light_x_axis.y, light_x_axis.z, 0.0f},
      {light_y_axis.x, light_y_axis.y, light_y_axis.z, 0.0f},
      {light_z_axis.x, light_z_axis.y, light_z_axis.z, 0.0f},
  };
  auto const camera_to_light_matrix =
      world_to_light_matrix *
      Mat4x4f{camera_to_world_matrix, {0.0f, 0.0f, 0.0f, 1.0f}};
  auto const uniform_buffer_data = _uniform_buffers.get().data();
  for (auto i = 0; i < cascade_count(); ++i) {
    auto const cascade_near_log = c_log(i);
    auto const cascade_near_uni = c_uni(i);
    auto const cascade_far_log = c_log(i + 1);
    auto const cascade_far_uni = c_uni(i + 1);
    auto const cascade_near =
        lambda * cascade_near_log + (1.0f - lambda) * cascade_near_uni;
    auto const cascade_far =
        lambda * cascade_far_log + (1.0f - lambda) * cascade_far_uni;
    auto const camera_space_frustum_vertices = std::array<Vec3f, 8>{
        Vec3f{-tan_half_fov.x * cascade_near,
              -tan_half_fov.y * cascade_near,
              -cascade_near},
        Vec3f{tan_half_fov.x * cascade_near,
              -tan_half_fov.y * cascade_near,
              -cascade_near},
        Vec3f{-tan_half_fov.x * cascade_near,
              tan_half_fov.y * cascade_near,
              -cascade_near},
        Vec3f{tan_half_fov.x * cascade_near,
              tan_half_fov.y * cascade_near,
              -cascade_near},
        Vec3f{-tan_half_fov.x * cascade_far,
              -tan_half_fov.y * cascade_far,
              -cascade_far},
        Vec3f{tan_half_fov.x * cascade_far,
              -tan_half_fov.y * cascade_far,
              -cascade_far},
        Vec3f{-tan_half_fov.x * cascade_far,
              tan_half_fov.y * cascade_far,
              -cascade_far},
        Vec3f{tan_half_fov.x * cascade_far,
              tan_half_fov.y * cascade_far,
              -cascade_far},
    };
    auto const light_space_frustum_vertices = std::array<Vec3f, 8>{
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[0], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[1], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[2], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[3], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[4], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[5], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[6], 1.0f},
        camera_to_light_matrix * Vec4f{camera_space_frustum_vertices[7], 1.0f},
    };
    auto light_space_cascade_bounds_xy =
        Aabb2f{light_space_frustum_vertices[0].xy()};
    auto light_space_cascade_min_z = light_space_frustum_vertices[0].z;
    for (auto i = 1; i < 8; ++i) {
      light_space_cascade_bounds_xy = merge(
          light_space_cascade_bounds_xy, light_space_frustum_vertices[i].xy());
      light_space_cascade_min_z =
          min(light_space_cascade_min_z, light_space_frustum_vertices[i].z);
    }
    auto light_space_cascade_max_z = light_space_cascade_min_z;
    // auto const sphere_distance =
    //     min(0.5f * (cascade_near + cascade_far) * (1.0f + tan_squared_alpha),
    //         cascade_far);
    // auto const sphere_radius =
    //     length(Vec2f{cascade_far * tan_alpha, cascade_far -
    //     sphere_distance});
    // auto const sphere_center_world_space =
    //     camera.position - camera_z_axis * sphere_distance;
    // auto const sphere_center_light_space =
    //     Vec3f{dot(sphere_center_world_space, light_x_axis),
    //           dot(sphere_center_world_space, light_y_axis),
    //           dot(sphere_center_world_space, light_z_axis)};
    // auto const view_space_cascade_bounds_xy =
    //     Aabb2f{sphere_center_light_space.xy() - Vec2f::all(sphere_radius),
    //            sphere_center_light_space.xy() + Vec2f::all(sphere_radius)};
    // auto const view_space_cascade_min_z =
    //     sphere_center_light_space.z - sphere_radius;
    // auto view_space_cascade_max_z = view_space_cascade_min_z;
    for (auto const surface : scene.surfaces()) {
      if (surface->shadow_casting) {
        auto const model_to_light_matrix =
            world_to_light_matrix *
            Mat4x4f{surface->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
        auto const &model_space_surface_bounds =
            static_cast<Surface_mesh const *>(surface->mesh)
                ->model_space_bounds();
        auto const light_space_surface_bounds_center =
            model_to_light_matrix *
            Vec4f{center(model_space_surface_bounds), 1.0f};
        auto const light_space_surface_bounds_half_extents =
            abs(model_to_light_matrix) *
            Vec4f{0.5f * extents(model_space_surface_bounds), 0.0f};
        auto const light_space_surface_bounds =
            Aabb3f{light_space_surface_bounds_center -
                       light_space_surface_bounds_half_extents,
                   light_space_surface_bounds_center +
                       light_space_surface_bounds_half_extents};
        if (overlaps(light_space_surface_bounds.xy(),
                     light_space_cascade_bounds_xy)) {
          light_space_cascade_max_z =
              max(light_space_cascade_max_z, light_space_surface_bounds.max.z);
        }
      }
    }
    auto const light_space_cascade_bounds = Aabb3f{
        Vec3f{light_space_cascade_bounds_xy.min, light_space_cascade_min_z},
        Vec3f{light_space_cascade_bounds_xy.max, light_space_cascade_max_z}};
    auto const jitter_x = rand() / (float)RAND_MAX - 0.5f;
    auto const jitter_y = rand() / (float)RAND_MAX - 0.5f;
    auto const view_projection_matrix =
        Mat3x4f::translation({2.0f * jitter_x / _texture_resolution,
                              2.0f * jitter_y / _texture_resolution,
                              0.0f}) *
        Mat4x4f::orthographic(light_space_cascade_bounds.min.x,
                              light_space_cascade_bounds.max.x,
                              light_space_cascade_bounds.min.y,
                              light_space_cascade_bounds.max.y,
                              light_space_cascade_bounds.max.z,
                              light_space_cascade_bounds.min.z) *
        Mat4x4f{world_to_light_matrix, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const &cascade = cascades()[i];
    std::memcpy(cascade.uniform_buffer().data(), &view_projection_matrix, 48);
    glBindFramebuffer(GL_FRAMEBUFFER, cascade.framebuffer());
    glViewport(0, 0, _texture_resolution, _texture_resolution);
    glClearDepth(0.0f);
    glClear(GL_DEPTH_BUFFER_BIT);
    glUseProgram(_intrinsic_state->shader_program());
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, cascade.uniform_buffer().get());
    for (auto const surface : scene.surfaces()) {
      if (surface->shadow_casting) {
        auto const mesh = static_cast<Surface_mesh const *>(surface->mesh);
        auto const &model_space_surface_bounds = mesh->model_space_bounds();
        auto const model_to_light_matrix =
            world_to_light_matrix *
            Mat4x4f{surface->transform, {0.0f, 0.0f, 0.0f, 1.0f}};
        auto const light_space_surface_bounds_center =
            model_to_light_matrix *
            Vec4f{center(model_space_surface_bounds), 1.0f};
        auto const light_space_surface_bounds_half_extents =
            abs(world_to_light_matrix) *
            Vec4f{0.5f * extents(model_space_surface_bounds), 0.0f};
        auto const light_space_surface_bounds =
            Aabb3f{light_space_surface_bounds_center -
                       light_space_surface_bounds_half_extents,
                   light_space_surface_bounds_center +
                       light_space_surface_bounds_half_extents};
        if (overlaps(light_space_surface_bounds, light_space_cascade_bounds)) {
          glBindBufferRange(
              GL_UNIFORM_BUFFER,
              1,
              surface_resource.uniform_buffer().get(),
              surface_resource.get_mapping(surface).uniform_buffer_offset,
              48);
          mesh->bind_vertex_array();
          mesh->draw();
        }
      }
    }
    auto const light_space_cascade_extents =
        extents(light_space_cascade_bounds.xy());
    auto const pixel_length =
        2.0f *
        max(light_space_cascade_extents.x, light_space_cascade_extents.y) /
        _texture_resolution;
    std::memcpy(uniform_buffer_data + 64 * i, &view_projection_matrix, 48);
    std::memcpy(uniform_buffer_data + 64 * i + 48, &cascade_far, 4);
    std::memcpy(uniform_buffer_data + 64 * i + 52, &pixel_length, 4);
  }
  auto const count = static_cast<int32_t>(cascade_count());
  std::memcpy(uniform_buffer_data + 64 * max_cascade_count, &count, 4);
}
} // namespace marlon::graphics::gl