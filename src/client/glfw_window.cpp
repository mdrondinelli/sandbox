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

Unique_glfw_window_ptr make_unique_glfw_window(int width, int height,
                                           const char *title,
                                           GLFWmonitor *monitor,
                                           GLFWwindow *share) {
  const auto window = glfwCreateWindow(width, height, title, monitor, share);
  if (window == nullptr) {
    const char *error_description;
    glfwGetError(&error_description);
    throw std::runtime_error{error_description};
  }
  return Unique_glfw_window_ptr{window};
}

#ifdef __clang__
#pragma clang diagnostic ignored "-Wunused-function"
#endif

TEST_CASE("Unique_glfw_window_ptr") {
  REQUIRE_THROWS(make_unique_glfw_window(800, 600, "title"));
  const Shared_glfw_instance instance;
  const auto window = make_unique_glfw_window(800, 600, "title");
  {
    auto a = make_unique_glfw_window(100, 100, "a");
    auto b = std::move(a);
    a = std::move(b);
  }
}
} // namespace client
} // namespace marlon