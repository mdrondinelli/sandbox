#include "glfw_window.h"

#include <stdexcept>

#include <GLFW/glfw3.h>

namespace marlon::platform {
void Glfw_window_deleter::operator()(GLFWwindow *ptr) const {
  glfwDestroyWindow(ptr);
}

Unique_glfw_window_ptr make_unique_glfw_window(int width,
                                               int height,
                                               const char *title,
                                               GLFWmonitor *monitor,
                                               GLFWwindow *share) {
  const auto window = glfwCreateWindow(width, height, title, monitor, share);
  if (window == nullptr) {
    const char *what;
    glfwGetError(&what);
    throw std::runtime_error{what};
  }
  return Unique_glfw_window_ptr{window};
}
} // namespace marlon::platform
