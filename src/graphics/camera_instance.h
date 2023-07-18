#ifndef MARLON_RENDERING_CAMERA_INSTANCE_H
#define MARLON_RENDERING_CAMERA_INSTANCE_H

namespace marlon {
namespace rendering {
class Camera;
class Scene_node;

struct Camera_instance_create_info {
  Camera *camera;
  Scene_node *scene_node;
};

class Camera_instance {};
} // namespace rendering
} // namespace marlon

#endif