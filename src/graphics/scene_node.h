#ifndef MARLON_GRAPHICS_SCENE_NODE_H
#define MARLON_GRAPHICS_SCENE_NODE_H

#include "../math/quat.h"
#include "../math/vec.h"

namespace marlon {
namespace graphics {
struct Scene_node_create_info {
  math::Vec3f translation{math::Vec3f::zero()};
  math::Quatf rotation{math::Quatf::identity()};
  float scale{1.0f};
};

class Scene_node {};
} // namespace graphics
} // namespace marlon

#endif