#ifndef MARLON_RENDERING_RENDER_STREAM_H
#define MARLON_RENDERING_RENDER_STREAM_H

namespace marlon {
namespace rendering {
class Scene;
class Camera_instance;
class Render_destination;

struct Render_stream_create_info {
  Scene *source_scene;
  Camera_instance *source_camera_instance;
  Render_destination *destination;
};

class Render_stream {};
} // namespace rendering
} // namespace marlon

#endif