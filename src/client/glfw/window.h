#ifndef MARLON_GLFW_WINDOW_H
#define MARLON_GLFW_WINDOW_H

#include <memory>

struct GLFWmonitor;
struct GLFWwindow;

namespace marlon {
namespace glfw {
struct Window_deleter {
  void operator()(GLFWwindow *ptr) const;
};

using Unique_window_ptr = std::unique_ptr<GLFWwindow, Window_deleter>;

Unique_window_ptr make_unique_window(int width, int height, const char *title,
                                     GLFWmonitor *monitor = nullptr,
                                     GLFWwindow *share = nullptr);
} // namespace glfw
} // namespace marlon

#endif