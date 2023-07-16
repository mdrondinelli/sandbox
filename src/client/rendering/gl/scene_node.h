#ifndef MARLON_RENDERING_GL_SCENE_NODE_H
#define MARLON_RENDERING_GL_SCENE_NODE_H

#include "../../../shared/math/mat.h"
#include "../scene_node.h"

namespace marlon {
namespace rendering {
struct Gl_scene_node : public Scene_node {
  friend class Gl_scene;
  friend class Gl_scene_diff;

public:
  class Impl {
  public:
    explicit Impl(Scene_node_create_info const &create_info) noexcept;

    void set_translation(math::Vec3f const &value) noexcept {
      _translation = value;
    }

    void set_rotation(math::Quatf const &value) noexcept { _rotation = value; }

    void set_scale(float value) noexcept { _scale = value; }

    void blend_translation(math::Vec3f const &value, float factor) noexcept {
      _translation = _translation * (1.0f - factor) + value * factor;
    }

    void blend_rotation(math::Quatf const &value, float factor) noexcept {
      _rotation = normalize(_rotation * (1.0f - factor) + value * factor);
    }

    void blend_scale(float value, float factor) {
      _scale = _scale * (1.0f - factor) + value * factor;
    }

    math::Mat3x4f calculate_model_matrix() const noexcept {
      return {{1.0f, 0.0f, 0.0f, _translation.x},
              {0.0f, 1.0f, 0.0f, _translation.y},
              {0.0f, 0.0f, 1.0f, _translation.z}};
    }

  private:
    math::Vec3f _translation;
    math::Quatf _rotation;
    float _scale;
  };

  explicit Gl_scene_node(Scene_node_create_info const &create_info) noexcept;

private:
  Impl _impl;
};
} // namespace rendering
} // namespace marlon

#endif