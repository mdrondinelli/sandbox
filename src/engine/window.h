#ifndef MARLON_ENGINE_WINDOW_H
#define MARLON_ENGINE_WINDOW_H

#include "../graphics/gl/window.h"
#include "../math/vec.h"
#include "glfw_window.h"

namespace marlon {
namespace engine {
struct Window_create_info {
  math::Vec2i extents;
  const char *title;
};

// Member functions, including constructor, must be called from the main thread
class Window : public graphics::Gl_window {
public:
  
  explicit Window(Window_create_info const &create_info);

  math::Vec2i get_framebuffer_extents() const noexcept final;

  GLFWwindow *get_glfw_window() noexcept;

private:
  Unique_glfw_window_ptr _glfw_window;
};
} // namespace engine
} // namespace marlon

#endif