#ifndef MARLON_GRAPHICS_SCENE_DIFF_H
#define MARLON_GRAPHICS_SCENE_DIFF_H

#include "../../math/quat.h"
#include "../../math/vec.h"

namespace marlon {
namespace graphics {
class Scene;

struct Scene_diff_create_info {
  Scene *scene;
};

class Scene_diff {};
} // namespace rendering
} // namespace marlon

#endif