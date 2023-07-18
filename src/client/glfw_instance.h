#ifndef MARLON_CLIENT_GLFW_INSTANCE_H
#define MARLON_CLIENT_GLFW_INSTANCE_H

namespace marlon {
namespace client {
class Glfw_instance {
public:
  Glfw_instance();
  ~Glfw_instance();

  Glfw_instance(Glfw_instance &&other) noexcept;
  Glfw_instance &operator=(Glfw_instance &&other) noexcept;

  bool owns() const noexcept;

private:
  void swap(Glfw_instance &other) noexcept;

  bool _owns{false};
};
} // namespace glfw
} // namespace marlon

#endif