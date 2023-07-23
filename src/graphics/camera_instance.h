#ifndef MARLON_GRAPHICS_CAMERA_INSTANCE_H
#define MARLON_GRAPHICS_CAMERA_INSTANCE_H

namespace marlon {
namespace graphics {
class Camera;
class Scene_node;

struct Camera_instance_create_info {
  Camera *camera;
  Scene_node *scene_node;
};

class Camera_instance {};
} // namespace graphics
} // namespace marlon

#endif