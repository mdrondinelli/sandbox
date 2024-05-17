#include "scene.h"

namespace marlon {
namespace graphics {
using namespace math;
using namespace util;

namespace {
constexpr std::size_t memory_requirement(std::size_t max_surfaces,
                                         std::size_t max_wireframes) noexcept {
  return Stack_allocator<>::memory_requirement({
      Set<Surface *>::memory_requirement(max_surfaces),
      Set<Wireframe *>::memory_requirement(max_wireframes),
  });
}
} // namespace

Scene::Scene(Scene_create_info const &create_info) noexcept
    : _memory{System_allocator::instance()->alloc(memory_requirement(
          create_info.max_surfaces, create_info.max_wireframes))} {
  auto allocator = Stack_allocator<>{_memory};
  _surfaces =
      decltype(_surfaces)::make(allocator, create_info.max_surfaces).second;
  _wireframes =
      decltype(_wireframes)::make(allocator, create_info.max_wireframes).second;
}

Scene::~Scene() {
  _wireframes = {};
  _surfaces = {};
  System_allocator::instance()->free(_memory);
}
}
}