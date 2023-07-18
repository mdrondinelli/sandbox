#include "glfw_window.h"

#include <stdexcept>

#include <GLFW/glfw3.h>
#include <catch2/catch_test_macros.hpp>

#include "glfw_instance.h"

namespace marlon {
namespace client {
void Glfw_window_deleter::operator()(GLFWwindow *ptr) const {
  glfwDestroyWindow(ptr);
}

Glfw_unique_window make_glfw_unique_window(int width, int height,
                                           const char *title,
                                           GLFWmonitor *monitor,
                                           GLFWwindow *share) {
  const auto window = glfwCreateWindow(width, height, title, monitor, share);
  if (window == nullptr) {
    const char *error_description;
    glfwGetError(&error_description);
    throw std::runtime_error{error_description};
  }
  return Glfw_unique_window{window};
}

TEST_CASE("glfw::Glfw_unique_window") {
  REQUIRE_THROWS(make_glfw_unique_window(800, 600, "title"));
  const Glfw_shared_instance instance;
  const auto window = make_glfw_unique_window(800, 600, "title");
  {
    auto a = make_glfw_unique_window(100, 100, "a");
    auto b = std::move(a);
    a = std::move(b);
  }
}
} // namespace client
} // namespace marlon