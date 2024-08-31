#include "scene.h"

namespace marlon {
namespace graphics {
using namespace math;
using namespace util;

namespace {
constexpr std::size_t memory_requirement(std::size_t max_surfaces) noexcept {
  return Stack_allocator<>::memory_requirement({Set<Surface *>::memory_requirement(max_surfaces)});
}
} // namespace

Scene::Scene(Scene_create_info const &create_info) noexcept
    : _memory{System_allocator{}.alloc(memory_requirement(create_info.max_surfaces))} {
  auto allocator = Stack_allocator<>{_memory};
  _surfaces = decltype(_surfaces)::make(allocator, create_info.max_surfaces).second;
}

Scene::~Scene() {
  _surfaces = {};
  System_allocator{}.free(_memory);
}
} // namespace graphics
} // namespace marlon
