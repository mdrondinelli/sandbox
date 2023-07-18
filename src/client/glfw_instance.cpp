#include "glfw_instance.h"

#include <stdexcept>
#include <utility>

#include <GLFW/glfw3.h>
#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace client {
namespace {
int instance_count{0};
}

Glfw_shared_instance::Glfw_shared_instance() {
  if (instance_count != 0) {
    throw std::runtime_error{
        "attempted to construct more than once simultaneous glfw instance"};
  }
  if (glfwInit() != GLFW_TRUE) {
    const char *error_description{nullptr};
    glfwGetError(&error_description);
    throw std::runtime_error{error_description};
  }
  _owns = true;
  ++instance_count;
}

Glfw_shared_instance::~Glfw_shared_instance() {
  if (_owns) {
    glfwTerminate();
    _owns = false;
    --instance_count;
  }
}

Glfw_shared_instance::Glfw_shared_instance(Glfw_shared_instance &&other) noexcept
    : _owns{std::exchange(other._owns, false)} {}

Glfw_shared_instance &Glfw_shared_instance::operator=(Glfw_shared_instance &&other) noexcept {
  auto temp{std::move(other)};
  swap(temp);
  return *this;
}

bool Glfw_shared_instance::owns() const noexcept { return _owns; }

void Glfw_shared_instance::swap(Glfw_shared_instance &other) noexcept { std::swap(_owns, other._owns); }

TEST_CASE("glfw::Instance") {
  REQUIRE(instance_count == 0);
  {
    Glfw_shared_instance instance;
    REQUIRE(instance_count == 1);
    REQUIRE(instance.owns());
    Glfw_shared_instance other_instance{std::move(instance)};
    REQUIRE(instance_count == 1);
    REQUIRE(!instance.owns());
    REQUIRE(other_instance.owns());
    REQUIRE_THROWS(Glfw_shared_instance{});
  }
  REQUIRE(instance_count == 0);
}
} // namespace glfw
} // namespace marlon