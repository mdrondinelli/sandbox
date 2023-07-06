#ifndef MARLON_RENDERING_SCENE_NODE_H
#define MARLON_RENDERING_SCENE_NODE_H

// #include <variant>

#include "../../shared/math/quat.h"
#include "../../shared/math/vec.h"

namespace marlon {
namespace rendering {
struct Scene_node_create_info {
  math::Vec3f position{0.0f, 0.0f, 0.0f};
  math::Quatf orientation{math::Quatf::identity()};
  float scale_factor{1.0f};
};

class Scene_node {
public:
  virtual ~Scene_node() = default;

  virtual math::Vec3f position() const noexcept = 0;

  virtual Scene_node &position(math::Vec3f const &new_position) noexcept = 0;

  virtual Scene_node &translate(math::Vec3f const &translation) noexcept = 0;

  virtual math::Quatf orientation() const noexcept = 0;

  virtual Scene_node &orientation(math::Quatf const &new_orientation) noexcept;

  virtual Scene_node &pre_rotate(math::Quatf const &rotation) noexcept;

  virtual Scene_node &post_rotate(math::Quatf const &rotation) noexcept;

  virtual float scale_factor() const noexcept;

  virtual Scene_node &scale_factor(float new_scale_factor) noexcept;

  virtual Scene_node &scale(float amount) noexcept;
};
} // namespace rendering
} // namespace marlon

#endif