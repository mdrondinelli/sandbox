#include "glfw_init_guard.h"

#include <stdexcept>
#include <utility>

#include <GLFW/glfw3.h>

namespace marlon {
namespace engine {
namespace {
int instance_count{0};
}

Glfw_init_guard::Glfw_init_guard() {
  if (instance_count != 0) {
    throw std::runtime_error{
        "attempted to construct more than once simultaneous glfw instance"};
  }
  if (glfwInit() != GLFW_TRUE) {
    const char *what{nullptr};
    glfwGetError(&what);
    throw std::runtime_error{what};
  }
  _owns = true;
  ++instance_count;
}

Glfw_init_guard::~Glfw_init_guard() {
  if (_owns) {
    glfwTerminate();
    _owns = false;
    --instance_count;
  }
}

Glfw_init_guard::Glfw_init_guard(Glfw_init_guard &&other) noexcept
    : _owns{std::exchange(other._owns, false)} {}

Glfw_init_guard &Glfw_init_guard::operator=(Glfw_init_guard &&other) noexcept {
  auto temp{std::move(other)};
  swap(temp);
  return *this;
}

void Glfw_init_guard::swap(Glfw_init_guard &other) noexcept {
  std::swap(_owns, other._owns);
}
} // namespace engine
} // namespace marlon
