#ifndef MARLON_GRAPHICS_SCENE_DIFF_H
#define MARLON_GRAPHICS_SCENE_DIFF_H

namespace marlon {
namespace graphics {
class Scene;

struct Scene_diff_create_info {
  Scene *scene{nullptr};
};

class Scene_diff {};
} // namespace rendering
} // namespace marlon

#endif