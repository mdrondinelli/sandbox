#include "Instance.h"

#include <stdexcept>
#include <utility>

#include <GLFW/glfw3.h>
#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace glfw {
namespace {
int instance_count{0};
}

Instance::Instance() {
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

Instance::~Instance() {
  if (_owns) {
    glfwTerminate();
    _owns = false;
    --instance_count;
  }
}

Instance::Instance(Instance &&other) noexcept
    : _owns{std::exchange(other._owns, false)} {}

Instance &Instance::operator=(Instance &&other) noexcept {
  auto temp{std::move(other)};
  swap(temp);
  return *this;
}

bool Instance::owns() const noexcept { return _owns; }

void Instance::swap(Instance &other) noexcept { std::swap(_owns, other._owns); }

TEST_CASE("glfw::Instance") {
  REQUIRE(instance_count == 0);
  {
    Instance instance;
    REQUIRE(instance_count == 1);
    REQUIRE(instance.owns());
    Instance other_instance{std::move(instance)};
    REQUIRE(instance_count == 1);
    REQUIRE(!instance.owns());
    REQUIRE(other_instance.owns());
    REQUIRE_THROWS(Instance{});
  }
  REQUIRE(instance_count == 0);
}
} // namespace glfw
} // namespace marlon