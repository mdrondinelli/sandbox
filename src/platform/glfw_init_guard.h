#ifndef MARLON_PLATFORM_GLFW_INIT_GUARD_H
#define MARLON_PLATFORM_GLFW_INIT_GUARD_H

namespace marlon::platform {
struct Glfw_init_guard_create_info {};

class Glfw_init_guard {
public:
  Glfw_init_guard() = default;

  Glfw_init_guard(Glfw_init_guard_create_info const &);

  ~Glfw_init_guard();

  Glfw_init_guard(Glfw_init_guard &&other) noexcept;

  Glfw_init_guard &operator=(Glfw_init_guard &&other) noexcept;

private:
  void swap(Glfw_init_guard &other) noexcept;

  bool _owns{false};
};
} // namespace marlon::platform

#endif
