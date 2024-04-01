#include "window.h"

#include <GLFW/glfw3.h>

using namespace marlon::math;

namespace marlon {
namespace engine {
Window::Window(Window_create_info const &create_info)
    : _glfw_window{[&]() {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        return make_unique_glfw_window(
            create_info.extents.x, create_info.extents.y, create_info.title);
      }()} {}

Vec2i Window::get_framebuffer_extents() const noexcept {
  auto result = Vec2i{};
  glfwGetFramebufferSize(_glfw_window.get(), &result[0], &result[1]);
  return result;
}

GLFWwindow *Window::get_glfw_window() noexcept {
  return _glfw_window.get();
}
} // namespace engine
} // namespace marlon