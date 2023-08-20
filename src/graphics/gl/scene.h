#ifndef MARLON_GRAPHICS_GL_SCENE_H
#define MARLON_GRAPHICS_GL_SCENE_H

#include <memory>

#include "../../math/mat.h"
#include "../scene.h"

namespace marlon {
namespace graphics {
class Gl_scene final : public Scene {
  friend class Gl_graphics;
  friend class Gl_scene_diff;

public:
  explicit Gl_scene(Scene_create_info const &create_info) noexcept;

  ~Gl_scene();

  void add_surface(Surface *surface) final;

  void remove_surface(Surface *surface) final;

private:
  struct Impl;

  void draw_surfaces(std::uint32_t shader_program,
                     std::uint32_t default_base_color_texture,
                     std::int32_t model_view_matrix_location,
                     std::int32_t model_view_clip_matrix_location,
                     std::int32_t base_color_tint_location,
                     math::Mat4x4f const &view_matrix,
                     math::Mat4x4f const &view_clip_matrix);

  std::unique_ptr<Impl> _impl;
};
} // namespace graphics
} // namespace marlon

#endif