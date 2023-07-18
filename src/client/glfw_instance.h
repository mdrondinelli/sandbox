#ifndef MARLON_CLIENT_GLFW_INSTANCE_H
#define MARLON_CLIENT_GLFW_INSTANCE_H

namespace marlon {
namespace client {
class Glfw_shared_instance {
public:
  Glfw_shared_instance();
  ~Glfw_shared_instance();

  Glfw_shared_instance(Glfw_shared_instance &&other) noexcept;
  Glfw_shared_instance &operator=(Glfw_shared_instance &&other) noexcept;

  bool owns() const noexcept;

private:
  void swap(Glfw_shared_instance &other) noexcept;

  bool _owns{false};
};
} // namespace glfw
} // namespace marlon

#endif