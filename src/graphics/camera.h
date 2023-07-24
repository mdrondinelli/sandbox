#ifndef MARLON_GRAPHICS_CAMERA_H
#define MARLON_GRAPHICS_CAMERA_H

namespace marlon {
namespace graphics {
struct Camera_create_info {
  float near_plane_distance{0.001f};
  float far_plane_distance{1000.0f};
  float zoom_x{1.0f};
  float zoom_y{1.0f};
};

class Camera {};
} // namespace graphics
} // namespace marlon

#endif