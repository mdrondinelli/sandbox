#include "scene.h"

#include <ankerl/unordered_dense.h>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#pragma clang diagnostic pop
#else
#include <glad/glad.h>
#endif

#include "material.h"
#include "mesh.h"
#include "surface.h"
#include "texture.h"

namespace marlon {
namespace graphics {

struct Gl_scene::Impl {
  void add_surface(Surface *surface) {
    surfaces.emplace(static_cast<Gl_surface *>(surface));
  }

  void remove_surface(Surface *surface) {
    surfaces.erase(static_cast<Gl_surface *>(surface));
  }

  void draw_surfaces(std::uint32_t shader_program,
                     std::uint32_t default_base_color_texture,
                     std::int32_t model_view_matrix_location,
                     std::int32_t model_view_clip_matrix_location,
                     std::int32_t base_color_tint_location,
                     math::Mat4x4f const &view_matrix,
                     math::Mat4x4f const &view_clip_matrix) {
    for (auto const surface_instance : surfaces) {
      auto const &model_matrix_3x4 = surface_instance->get_transform();
      auto const model_matrix_4x4 = math::Mat4x4f{model_matrix_3x4[0],
                                                  model_matrix_3x4[1],
                                                  model_matrix_3x4[2],
                                                  {0.0f, 0.0f, 0.0f, 1.0f}};
      auto const model_view_matrix = view_matrix * model_matrix_4x4;
      auto const model_view_clip_matrix = view_clip_matrix * model_matrix_4x4;
      glProgramUniformMatrix4fv(shader_program, model_view_matrix_location, 1,
                                GL_TRUE, &model_view_matrix[0][0]);
      glProgramUniformMatrix4fv(shader_program, model_view_clip_matrix_location,
                                1, GL_TRUE, &model_view_clip_matrix[0][0]);
      auto const material = surface_instance->get_material();
      if (auto const base_color_texture =
              material->_impl.get_base_color_texture()) {
        glBindTextureUnit(0, base_color_texture->_handle.get());
      } else {
        glBindTextureUnit(0, default_base_color_texture);
      }
      glProgramUniform3f(shader_program, base_color_tint_location,
                         material->_impl.get_base_color_tint().r,
                         material->_impl.get_base_color_tint().g,
                         material->_impl.get_base_color_tint().b);
      auto const mesh = surface_instance->get_mesh();
      mesh->_impl.bind_vertex_array();
      mesh->_impl.draw();
    }
  }

  ankerl::unordered_dense::set<Gl_surface *> surfaces;
};

Gl_scene::Gl_scene(Scene_create_info const &) noexcept
    : _impl{std::make_unique<Impl>()} {}

Gl_scene::~Gl_scene() {}

void Gl_scene::add_surface(Surface *surface) {
  _impl->add_surface(surface);
}

void Gl_scene::remove_surface(Surface *surface) {
  _impl->remove_surface(surface);
}

void Gl_scene::draw_surfaces(std::uint32_t shader_program,
                             std::uint32_t default_base_color_texture,
                             std::int32_t model_view_matrix_location,
                             std::int32_t model_view_clip_matrix_location,
                             std::int32_t base_color_tint_location,
                             math::Mat4x4f const &view_matrix,
                             math::Mat4x4f const &view_clip_matrix) {
  _impl->draw_surfaces(shader_program, default_base_color_texture,
                       model_view_matrix_location,
                       model_view_clip_matrix_location,
                       base_color_tint_location, view_matrix, view_clip_matrix);
}
} // namespace graphics
} // namespace marlon