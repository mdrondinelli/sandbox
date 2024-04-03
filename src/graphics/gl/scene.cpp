#include "scene.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

#include "../../util/set.h"
#include "surface.h"
#include "texture.h"

namespace marlon {
namespace graphics {
namespace {
constexpr std::size_t memory_requirement(std::size_t max_surfaces) noexcept {
  return util::Stack_allocator<>::memory_requirement({
      util::Pool<Gl_surface>::memory_requirement(max_surfaces),
      util::Set<Gl_surface *>::memory_requirement(max_surfaces),
  });
}
} // namespace

Gl_scene::Gl_scene(Scene_create_info const &create_info) noexcept
    : _memory{util::System_allocator::instance()->alloc(
          memory_requirement(create_info.max_surfaces))} {
  auto allocator = util::Stack_allocator<>{_memory};
  _surface_pool =
      util::make_pool<Gl_surface>(allocator, create_info.max_surfaces).second;
  _surfaces =
      util::make_set<Gl_surface *>(allocator, create_info.max_surfaces).second;
}

Gl_scene::~Gl_scene() {
  _surfaces = {};
  _surface_pool = {};
  util::System_allocator::instance()->free(_memory);
}

Surface *Gl_scene::create_surface(Surface_create_info const &create_info) {
  auto const surface = _surface_pool.emplace(create_info);
  _surfaces.emplace(surface);
  return surface;
}

void Gl_scene::destroy_surface(Surface *surface) noexcept {
  auto const gl_surface = static_cast<Gl_surface *>(surface);
  _surfaces.erase(gl_surface);
  _surface_pool.erase(gl_surface);
}

void Gl_scene::draw_surfaces(std::uint32_t shader_program,
                             std::uint32_t default_base_color_texture,
                             std::int32_t model_view_matrix_location,
                             std::int32_t model_view_clip_matrix_location,
                             std::int32_t base_color_tint_location,
                             math::Mat4x4f const &view_matrix,
                             math::Mat4x4f const &view_clip_matrix) {
  for (auto const surface : _surfaces) {
    auto const &model_matrix_3x4 = surface->get_transform();
    auto const model_matrix_4x4 =
        math::Mat4x4f{model_matrix_3x4, {0.0f, 0.0f, 0.0f, 1.0f}};
    auto const model_view_matrix = view_matrix * model_matrix_4x4;
    auto const model_view_clip_matrix = view_clip_matrix * model_matrix_4x4;
    glProgramUniformMatrix4fv(shader_program,
                              model_view_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_matrix[0][0]);
    glProgramUniformMatrix4fv(shader_program,
                              model_view_clip_matrix_location,
                              1,
                              GL_TRUE,
                              &model_view_clip_matrix[0][0]);
    auto const material = surface->get_material();
    if (auto const base_color_texture =
            material->_impl.get_base_color_texture()) {
      glBindTextureUnit(0, base_color_texture->_handle.get());
    } else {
      glBindTextureUnit(0, default_base_color_texture);
    }
    glProgramUniform3f(shader_program,
                       base_color_tint_location,
                       material->_impl.get_base_color_tint().r,
                       material->_impl.get_base_color_tint().g,
                       material->_impl.get_base_color_tint().b);
    auto const mesh = surface->get_mesh();
    mesh->bind_vertex_array();
    mesh->draw();
  }
}
} // namespace graphics
} // namespace marlon