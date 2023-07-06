#ifndef MARLON_RENDERING_CAMERA_INSTANCE_H
#define MARLON_RENDERING_CAMERA_INSTANCE_H

namespace marlon {
namespace rendering {
class Camera;
class Scene_node;

struct Camera_instance_create_info {
  Camera *camera;
  Scene_node *node;
};

class Camera_instance {
public:
  virtual ~Camera_instance() = default;

  virtual Camera *camera() const noexcept = 0;

  virtual Scene_node *node() const noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif