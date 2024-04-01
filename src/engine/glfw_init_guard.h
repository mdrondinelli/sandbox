#ifndef MARLON_ENGINE_GLFW_INIT_GUARD_H
#define MARLON_ENGINE_GLFW_INIT_GUARD_H

namespace marlon {
namespace engine {
class Glfw_init_guard {
public:
  Glfw_init_guard();
  ~Glfw_init_guard();

  Glfw_init_guard(Glfw_init_guard &&other) noexcept;
  Glfw_init_guard &operator=(Glfw_init_guard &&other) noexcept;

private:
  void swap(Glfw_init_guard &other) noexcept;

  bool _owns{false};
};
} // namespace engine
} // namespace marlon

#endif