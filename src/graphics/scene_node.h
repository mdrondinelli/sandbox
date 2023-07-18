#ifndef MARLON_RENDERING_SCENE_NODE_H
#define MARLON_RENDERING_SCENE_NODE_H

#include "../../math/quat.h"
#include "../../math/vec.h"

namespace marlon {
namespace graphics {
struct Scene_node_create_info {
  math::Vec3f translation;
  math::Quatf rotation;
  float scale;
};

class Scene_node {};
} // namespace rendering
} // namespace marlon

#endif