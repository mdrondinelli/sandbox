#ifndef MARLON_RENDERING_RENDER_GROUP_H
#define MARLON_RENDERING_RENDER_GROUP_H

#include <span>

namespace marlon {
namespace rendering {
class Camera_instance;
class Render_destination;

struct Render_info {
  Camera_instance *source;
  Render_destination *destination;
};

struct Render_group_create_info {
  std::span<Render_info const> render_infos;
};

class Render_group {
public:
  virtual ~Render_group() = default;

  virtual void render() = 0;
};
} // namespace rendering
} // namespace marlon

#endif