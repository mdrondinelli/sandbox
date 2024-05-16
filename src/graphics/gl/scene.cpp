#include "scene.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

#include <util/set.h>

#include "surface.h"
#include "texture.h"

namespace marlon {
namespace graphics {
namespace gl {
using namespace math;
using namespace util;

namespace {
constexpr std::size_t memory_requirement(std::size_t max_surfaces,
                                         std::size_t max_wireframes) noexcept {
  return Stack_allocator<>::memory_requirement({
      Pool<Surface>::memory_requirement(max_surfaces),
      Pool<Wireframe>::memory_requirement(max_wireframes),
      Set<Surface *>::memory_requirement(max_surfaces),
      Set<Wireframe *>::memory_requirement(max_wireframes),
  });
}
} // namespace

Scene::Scene(Scene_create_info const &create_info) noexcept
    : _memory{System_allocator::instance()->alloc(memory_requirement(
          create_info.max_surfaces, create_info.max_wireframes))} {
  auto allocator = Stack_allocator<>{_memory};
  _surface_pool =
      Pool<Surface>::make(allocator, create_info.max_surfaces).second;
  _wireframe_pool =
      Pool<Wireframe>::make(allocator, create_info.max_wireframes).second;
  _surfaces = Set<Surface *>::make(allocator, create_info.max_surfaces).second;
  _wireframes =
      Set<Wireframe *>::make(allocator, create_info.max_wireframes).second;
}

Scene::~Scene() {
  _wireframes = {};
  _surfaces = {};
  _wireframe_pool = {};
  _surface_pool = {};
  System_allocator::instance()->free(_memory);
}

Rgb_spectrum Scene::get_ambient_irradiance() const noexcept {
  return _ambient_irradiance;
}

void Scene::set_ambient_irradiance(Rgb_spectrum ambient_irradiance) noexcept {
  _ambient_irradiance = ambient_irradiance;
}

std::optional<Directional_light> const &
Scene::get_directional_light() const noexcept {
  return _directional_light;
}

void Scene::set_directional_light(
    std::optional<Directional_light> const &directional_light) noexcept {
  _directional_light = directional_light;
}

Surface *Scene::create_surface(Surface_create_info const &create_info) {
  auto const result = _surface_pool.emplace(create_info);
  _surfaces.emplace(result);
  return result;
}

void Scene::destroy_surface(graphics::Surface *surface) noexcept {
  auto const gl_surface = static_cast<Surface *>(surface);
  _surfaces.erase(gl_surface);
  _surface_pool.erase(gl_surface);
}

Wireframe *Scene::create_wireframe(Wireframe_create_info const &create_info) {
  auto const result = _wireframe_pool.emplace(create_info);
  _wireframes.emplace(result);
  return result;
}

void Scene::destroy_wireframe(graphics::Wireframe *wireframe) noexcept {
  auto const gl_wireframe = static_cast<Wireframe *>(wireframe);
  _wireframes.erase(gl_wireframe);
  _wireframe_pool.erase(gl_wireframe);
}

void Scene::draw_surfaces(std::uint32_t shader_program,
                          std::uint32_t default_base_color_texture,
                          std::int32_t model_matrix_location,
                          std::int32_t model_view_clip_matrix_location,
                          std::int32_t base_color_tint_location,
                          std::int32_t ambient_irradiance_location,
                          std::int32_t directional_light_irradiance_location,
                          std::int32_t directional_light_direction_location,
                          std::int32_t exposure_location,
                          Mat4x4f const &view_clip_matrix,
                          float exposure) const {
  glUseProgram(shader_program);
  glProgramUniform3f(shader_program,
                     ambient_irradiance_location,
                     _ambient_irradiance.r,
                     _ambient_irradiance.g,
                     _ambient_irradiance.b);
  if (_directional_light) {
    glProgramUniform3f(shader_program,
                       directional_light_irradiance_location,
                       _directional_light->irradiance.r,
                       _directional_light->irradiance.g,
                       _directional_light->irradiance.b);
    glProgramUniform3f(shader_program,
                       directional_light_direction_location,
                       _directional_light->direction.x,
                       _directional_light->direction.y,
                       _directional_light->direction.z);
  } else {
    glProgramUniform3f(shader_program,
                       directional_light_irradiance_location,
                       0.0f,
                       0.0f,
                       0.0f);
    glProgramUniform3f(
        shader_program, directional_light_direction_location, 1.0f, 0.0f, 0.0f);
  }
  glProgramUniform1f(shader_program, exposure_location, exposure);
  for (auto const surface : _surfaces) {
    auto const &model_matrix_3x4 = surface->get_transform();
    auto const model_matrix_4x4 =
        Mat4x4f{model_matrix_3x4, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const model_view_clip_matrix = view_clip_matrix * model_matrix_4x4;
    glProgramUniformMatrix4fv(shader_program,
                              model_matrix_location,
                              1,
                              GL_TRUE,
                              &model_matrix_4x4[0][0]);
    glProgramUniformMatrix4fv(shader_program,
                              model_view_clip_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_clip_matrix[0][0]);
    auto const &material = surface->get_material();
    if (auto const base_color_texture =
            static_cast<Texture *>(material.base_color_texture)) {
      glBindTextureUnit(0, base_color_texture->_handle.get());
    } else {
      glBindTextureUnit(0, default_base_color_texture);
    }
    glProgramUniform3f(shader_program,
                       base_color_tint_location,
                       material.base_color_tint.r,
                       material.base_color_tint.g,
                       material.base_color_tint.b);
    auto const mesh = surface->get_mesh();
    mesh->bind_vertex_array();
    mesh->draw();
  }
}

void Scene::draw_wireframes(std::uint32_t shader_program,
                            std::int32_t model_view_clip_matrix_location,
                            std::int32_t color_location,
                            Mat4x4f const &view_clip_matrix) const {
  glUseProgram(shader_program);
  for (auto const wireframe : _wireframes) {
    auto const &model_matrix_3x4 = wireframe->get_transform();
    auto const model_matrix_4x4 =
        Mat4x4f{model_matrix_3x4, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const model_view_clip_matrix = view_clip_matrix * model_matrix_4x4;
    glProgramUniformMatrix4fv(shader_program,
                              model_view_clip_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_clip_matrix[0][0]);
    glProgramUniform3f(shader_program,
                       color_location,
                       wireframe->get_color().r,
                       wireframe->get_color().g,
                       wireframe->get_color().b);
    auto const mesh = wireframe->get_mesh();
    mesh->bind_vertex_array();
    mesh->draw();
  }
}
} // namespace gl
} // namespace graphics
} // namespace marlon