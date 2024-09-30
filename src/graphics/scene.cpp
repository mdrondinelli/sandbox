#include "scene.h"

#include <utility>

namespace marlon {
namespace graphics {
using namespace math;
using namespace util;

namespace {
constexpr std::size_t memory_requirement(std::size_t max_surfaces) noexcept {
  return Stack_allocator<>::memory_requirement(
      {Set<Surface *>::memory_requirement(max_surfaces)});
}
} // namespace

Scene::Scene(Scene_create_info const &create_info) noexcept
    : _memory{System_allocator{}.alloc(
          memory_requirement(create_info.max_surfaces))} {
  auto allocator = Stack_allocator<>{_memory};
  _surfaces =
      decltype(_surfaces)::make(allocator, create_info.max_surfaces).second;
}

Scene::~Scene() {
  _surfaces = {};
  System_allocator{}.free(_memory);
}

Scene::Scene(Scene &&other) noexcept
    : _memory{std::exchange(other._memory, Block{})},
      _surfaces{std::move(other._surfaces)},
      _sun{std::move(other._sun)},
      _sky_irradiance{std::move(other._sky_irradiance)},
      _ground_albedo{std::move(other._ground_albedo)} {}

Scene &Scene::operator=(Scene &&other) noexcept {
  auto temp{std::move(other)};
  swap(temp);
  return *this;
}

void Scene::swap(Scene &other) noexcept {
  std::swap(_memory, other._memory);
  std::swap(_surfaces, other._surfaces);
  std::swap(_sun, other._sun);
  std::swap(_sky_irradiance, other._sky_irradiance);
  std::swap(_ground_albedo, other._ground_albedo);
}
} // namespace graphics
} // namespace marlon
