#include "window.h"

#include <stdexcept>

#include <GLFW/glfw3.h>
#include <catch2/catch_test_macros.hpp>

#include "instance.h"

namespace marlon {
namespace glfw {
void Window_deleter::operator()(GLFWwindow *ptr) const {
  glfwDestroyWindow(ptr);
}

Unique_window_ptr make_unique_window(int width, int height, const char *title,
                                     GLFWmonitor *monitor, GLFWwindow *share) {
  const auto window = glfwCreateWindow(width, height, title, monitor, share);
  if (window == nullptr) {
    const char *error_description;
    glfwGetError(&error_description);
    throw std::runtime_error{error_description};
  }
  return Unique_window_ptr{window};
}

TEST_CASE("glfw::Unique_window_ptr") {
  REQUIRE_THROWS(make_unique_window(800, 600, "title"));
  const Instance instance;
  const auto window = make_unique_window(800, 600, "title");
  {
    auto a = make_unique_window(100, 100, "a");
    auto b = std::move(a);
    a = std::move(b);
  }
}
} // namespace glfw
} // namespace marlon