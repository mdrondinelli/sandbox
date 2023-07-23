#ifndef MARLON_GRAPHICS_CAMERA_H
#define MARLON_GRAPHICS_CAMERA_H

namespace marlon {
namespace graphics {
struct Camera_create_info {
  float near_plane_distance;
  float far_plane_distance;
  float zoom_x;
  float zoom_y;
};

class Camera {};
} // namespace graphics
} // namespace marlon

#endif