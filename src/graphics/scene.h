#ifndef MARLON_GRAPHICS_SCENE_H
#define MARLON_GRAPHICS_SCENE_H

#include "surface.h"

namespace marlon {
namespace graphics {
struct Scene_create_info {};

class Scene {
public:
  virtual void add_surface(Surface *surface) = 0;
  
  virtual void remove_surface(Surface *surface) = 0;
};
} // namespace graphics
} // namespace marlon

#endif