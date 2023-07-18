#ifndef MARLON_RENDERING_SCENE_KEYFRAME_H
#define MARLON_RENDERING_SCENE_KEYFRAME_H

#include "../../math/quat.h"
#include "../../math/vec.h"

namespace marlon {
namespace rendering {
class Scene;

struct Scene_diff_create_info {
  Scene *scene;
};

class Scene_diff {};
} // namespace rendering
} // namespace marlon

#endif