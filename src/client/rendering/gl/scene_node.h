#ifndef MARLON_RENDERING_GL_SCENE_NODE_H
#define MARLON_RENDERING_GL_SCENE_NODE_H

#include "../scene_node.h"

namespace marlon {
namespace rendering {
class Gl_scene_node : public Scene_node {
public:
  explicit Gl_scene_node(Scene_node_create_info const &create_info) noexcept;

  math::Vec3f position() const noexcept override;

  Gl_scene_node &position(math::Vec3f const &position) noexcept override;

  Gl_scene_node &translate(math::Vec3f const &translation) noexcept override;

  math::Quatf orientation() const noexcept override;

  Gl_scene_node &orientation(math::Quatf const &orientation) noexcept override;

  Gl_scene_node &pre_rotate(math::Quatf const &rotation) noexcept override;

  Gl_scene_node &post_rotate(math::Quatf const &rotation) noexcept override;

  float scale_factor() const noexcept override;

  Gl_scene_node &scale_factor(float scale_factor) noexcept override;

  Gl_scene_node &scale(float amount) noexcept override;

private:
  math::Vec3f _position{0.0f, 0.0f, 0.0f};
  math::Quatf _orientation{math::Quatf::identity()};
  float _scale_factor{1.0f};
};
} // namespace rendering
} // namespace marlon

#endif