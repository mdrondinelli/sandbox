#ifndef MARLON_CLIENT_GLFW_WINDOW_H
#define MARLON_CLIENT_GLFW_WINDOW_H

#include <memory>

struct GLFWmonitor;
struct GLFWwindow;

namespace marlon {
namespace client {
struct Glfw_window_deleter {
  void operator()(GLFWwindow *ptr) const;
};

using Unique_glfw_window_ptr = std::unique_ptr<GLFWwindow, Glfw_window_deleter>;

Unique_glfw_window_ptr make_unique_window(int width, int height,
                                          const char *title,
                                          GLFWmonitor *monitor = nullptr,
                                          GLFWwindow *share = nullptr);
} // namespace client
} // namespace marlon

#endif