#include "render_stream.h"

#include <cstring>

#include <glad/gl.h>

#include <math/math.h>

#include "graphics/gl/render_target.h"
#include "graphics/gl/shaders/fullscreen.h"
#include "graphics/gl/shaders/lighting.h"
#include "graphics/gl/shaders/postprocessing.h"
#include "graphics/gl/shaders/surface.h"
#include "graphics/gl/shaders/temporal_antialiasing.h"
#include "graphics/gl/surface_mesh.h"

namespace marlon::graphics::gl {
using namespace math;
using namespace util;

namespace {
Mat4x4f perspective(Vec2f const &zoom, float near_plane_distance) {
  return Mat4x4f{{zoom.x, 0.0f, 0.0f, 0.0f},
                 {0.0f, -zoom.y, 0.0f, 0.0f},
                 {0.0f, 0.0f, 0.0f, near_plane_distance},
                 {0.0f, 0.0f, -1.0f, 0.0f}};
}
} // namespace

Render_stream::Intrinsic_state::Intrinsic_state(Intrinsic_state_create_info const &)
    : _cascaded_shadow_map_intrinsic_state{{}},
      _surface_shader_program{
          wrappers::make_unique_shader_program(surface_vertex_shader_source, surface_fragment_shader_source)},
      _lighting_shader_program{
          wrappers::make_unique_shader_program(fullscreen_vertex_shader_source, lighting_fragment_shader_source)},
      _temporal_antialiasing_shader_program{wrappers::make_unique_shader_program(
          fullscreen_vertex_shader_source, temporal_antialiasing_fragment_shader_source)},
      _postprocessing_shader_program{
          wrappers::make_unique_shader_program(fullscreen_vertex_shader_source, postprocessing_fragment_shader_source)},
      _empty_vertex_array{wrappers::make_unique_vertex_array()} {}

Render_stream::Render_stream(Intrinsic_state const *intrinsic_state,
                             Render_stream_create_info const &create_info) noexcept
    : _intrinsic_state{intrinsic_state},
      _target{static_cast<Render_target *>(create_info.target)},
      _scene{static_cast<Scene const *>(create_info.scene)},
      _camera{create_info.camera},
      _lighting_uniform_buffer{Uniform_buffer_create_info{.size = 132}} {}

void Render_stream::render() {
  auto const target_extents = _target->get_extents();
  if (target_extents == Vec2i::zero()) {
    return;
  }
  auto const frame_time = Clock::now();
  auto const taa_blend_factor = [&] {
    if (_frame_time) {
      auto const frame_duration = std::chrono::duration_cast<std::chrono::duration<float>>(frame_time - *_frame_time);
      return 1.0f - pow(1.0f - 0.1f, 60.0f * frame_duration.count());
    } else {
      return 1.0f;
    }
  }();
  _frame_time = frame_time;
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(0.0f);
  glClipControl(GL_UPPER_LEFT, GL_ZERO_TO_ONE);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CW);
  glDepthFunc(GL_GREATER);
  glEnable(GL_FRAMEBUFFER_SRGB);
  auto const inverse_view_matrix = Mat4x4f::rigid(_camera->position, _camera->orientation);
  auto const view_matrix = rigid_inverse(inverse_view_matrix);
  auto const projection_matrix = perspective(_camera->zoom, _camera->near_plane_distance);
  auto const view_projection_matrix = projection_matrix * view_matrix;
  acquire_surface_resource();
  write_surface_resource(view_projection_matrix);
  glEnable(GL_DEPTH_TEST);
  acquire_cascaded_shadow_map();
  draw_cascaded_shadow_map();
  auto const ndc_pixel_extents = Vec2f{2.0f / target_extents.x, 2.0f / target_extents.y};
  auto const jitter = Vec2f{(rand() / (float)RAND_MAX - 0.5f) * ndc_pixel_extents.x,
                            (rand() / (float)RAND_MAX - 0.5f) * ndc_pixel_extents.y};
  glViewport(0, 0, target_extents.x, target_extents.y);
  draw_visibility_buffer(Mat4x4f::translation(Vec3f{jitter, 0.0f}));
  release_surface_resource();
  glDisable(GL_DEPTH_TEST);
  glBindVertexArray(_intrinsic_state->empty_vertex_array());
  do_lighting(inverse_view_matrix);
  release_cascaded_shadow_map();
  do_temporal_antialiasing(taa_blend_factor);
  do_postprocessing();
  _taa_resource.swap_accumulation_buffers();
  ++_frame_number;
}

void Render_stream::acquire_surface_resource() {
  if (_surface_resource.max_surfaces() != _scene->surfaces().max_size()) {
    _surface_resource = Surface_resource{{
        .max_surfaces = _scene->surfaces().max_size(),
    }};
  }
  _surface_resource.acquire();
}

void Render_stream::release_surface_resource() {
  _surface_resource.release();
}

void Render_stream::write_surface_resource(Mat4x4f const &view_projection_matrix) {
  for (auto const surface : _scene->surfaces()) {
    _surface_resource.write(surface, view_projection_matrix);
  }
}

void Render_stream::acquire_cascaded_shadow_map() {
  if (_camera->csm_cascade_count <= 0) {
    _cascaded_shadow_map = {};
  } else if (_cascaded_shadow_map.cascade_count() != _camera->csm_cascade_count ||
             _cascaded_shadow_map.texture_resolution() != _camera->csm_texture_resolution) {
    _cascaded_shadow_map = Cascaded_shadow_map{
        _intrinsic_state->cascaded_shadow_map_intrinsic_state(),
        {
            .texture_resolution = _camera->csm_texture_resolution,
            .cascade_count = _camera->csm_cascade_count,
        },
    };
  }
  if (_cascaded_shadow_map) {
    _cascaded_shadow_map.acquire();
  }
}

void Render_stream::release_cascaded_shadow_map() {
  if (_cascaded_shadow_map) {
    _cascaded_shadow_map.release();
  }
}

void Render_stream::draw_cascaded_shadow_map() {
  if (_scene->sun() && _camera->csm_cascade_count > 0) {
    _cascaded_shadow_map.draw(*_scene, *_camera, _scene->sun()->direction, _surface_resource);
  }
}

void Render_stream::draw_visibility_buffer(Mat4x4f const &jitter_matrix) {
  auto const target_extents = _target->get_extents();
  if (_visibility_buffer.extents() != target_extents) {
    _visibility_buffer = Visibility_buffer{{.extents = target_extents}};
  }
  glBindFramebuffer(GL_FRAMEBUFFER, _visibility_buffer.framebuffer());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  draw_surfaces(jitter_matrix);
}

void Render_stream::draw_surfaces(Mat4x4f const &jitter_matrix) {
  auto constexpr jitter_matrix_location = 0;
  auto const shader_program = _intrinsic_state->surface_shader_program();
  glUseProgram(shader_program);
  glProgramUniformMatrix4fv(shader_program, jitter_matrix_location, 1, GL_TRUE, &jitter_matrix[0][0]);
  for (auto const surface : _scene->surfaces()) {
    if (!surface->visible) {
      continue;
    }
    glBindBufferRange(
        GL_UNIFORM_BUFFER, 0, _surface_resource.uniform_buffer(), _surface_resource.uniform_buffer_offset(surface), 64);
    auto const mesh = static_cast<Surface_mesh const *>(surface->mesh);
    mesh->bind_vertex_array();
    mesh->draw();
  }
}

void Render_stream::do_lighting(Mat4x4f const &inverse_view_matrix) {
  _lighting_uniform_buffer.acquire();
  auto const data = _lighting_uniform_buffer.get().data();
  std::memcpy(data, &inverse_view_matrix, 48);
  auto const tan_half_fov = Vec2f{1.0f / _camera->zoom.x, 1.0f / _camera->zoom.y};
  std::memcpy(data + 48, &tan_half_fov, 8);
  std::memcpy(data + 56, &_camera->near_plane_distance, 4);
  auto const sky_irradiance = _scene->sky_irradiance();
  std::memcpy(data + 64, &sky_irradiance, 12);
  auto const ground_albedo = _scene->ground_albedo();
  std::memcpy(data + 80, &ground_albedo, 12);
  if (auto const &directional_light = _scene->sun()) {
    std::memcpy(data + 96, &directional_light->irradiance, 12);
    std::memcpy(data + 112, &directional_light->direction, 12);
  } else {
    auto const v = Vec3f::zero();
    std::memcpy(data + 96, &v, 12);
    std::memcpy(data + 112, &v, 12);
  }
  std::memcpy(data + 128, &_frame_number, 4);
  auto const target_extents = _target->get_extents();
  if (_taa_resource.extents() != target_extents) {
    _taa_resource = Temporal_antialiasing_resource{{.extents = target_extents}};
  }
  auto const shader_program = _intrinsic_state->lighting_shader_program();
  glBindFramebuffer(GL_FRAMEBUFFER, _taa_resource.sample_buffer().framebuffer());
  glClear(GL_COLOR_BUFFER_BIT);
  glUseProgram(shader_program);
  glBindTextureUnit(0, _visibility_buffer.depth_texture());
  glBindTextureUnit(1, _visibility_buffer.color_texture());
  glBindTextureUnit(2, _visibility_buffer.normal_texture());
  glBindTextureUnit(3, _cascaded_shadow_map.texture());
  glBindBufferBase(GL_UNIFORM_BUFFER, 0, _lighting_uniform_buffer.get().get());
  glBindBufferBase(GL_UNIFORM_BUFFER, 1, _cascaded_shadow_map.uniform_buffer());
  glDrawArrays(GL_TRIANGLES, 0, 3);
  _lighting_uniform_buffer.release();
}

void Render_stream::do_temporal_antialiasing(float blend_factor) {
  glBindFramebuffer(GL_FRAMEBUFFER, _taa_resource.curr_accumulation_buffer().framebuffer());
  auto const shader_program = _intrinsic_state->temporal_antialiasing_shader_program();
  glUseProgram(shader_program);
  glBindTextureUnit(0, _taa_resource.prev_accumulation_buffer().texture());
  glBindTextureUnit(1, _visibility_buffer.depth_texture());
  glBindTextureUnit(2, _visibility_buffer.motion_vector_texture());
  glBindTextureUnit(3, _taa_resource.sample_buffer().texture());
  glProgramUniform1f(shader_program, 0, blend_factor);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

void Render_stream::do_postprocessing() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glUseProgram(_intrinsic_state->postprocessing_shader_program());
  glBindTextureUnit(0, _taa_resource.curr_accumulation_buffer().texture());
  glProgramUniform1f(_intrinsic_state->postprocessing_shader_program(), 0, _camera->exposure);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}
} // namespace marlon::graphics::gl
