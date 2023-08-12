#ifndef MARLON_CLIENT_GLFW_INSTANCE_H
#define MARLON_CLIENT_GLFW_INSTANCE_H

namespace marlon {
namespace client {
class Shared_glfw_instance {
public:
  Shared_glfw_instance();
  ~Shared_glfw_instance();

  Shared_glfw_instance(Shared_glfw_instance &&other) noexcept;
  Shared_glfw_instance &operator=(Shared_glfw_instance &&other) noexcept;

  bool owns() const noexcept;

private:
  void swap(Shared_glfw_instance &other) noexcept;

  bool _owns{false};
};
} // namespace glfw
} // namespace marlon

#endif