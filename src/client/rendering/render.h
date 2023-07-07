#ifndef MARLON_RENDERING_RENDER_H
#define MARLON_RENDERING_RENDER_H

namespace marlon {
namespace rendering {
class Camera_instance;
class Render_destination;

struct Render_create_info {
  Camera_instance *source;
  Render_destination *destination;
};

class Render {
public:
  virtual ~Render() = default;

  virtual Camera_instance *source() const noexcept = 0;
  
  virtual Render_destination *destination() const noexcept = 0;
};
} // namespace rendering
} // namespace marlon

#endif