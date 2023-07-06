#ifndef MARLON_GLFW_INSTANCE_H
#define MARLON_GLFW_INSTANCE_H

namespace marlon {
namespace glfw {
class Instance {
public:
  Instance();
  ~Instance();

  Instance(Instance &&other) noexcept;
  Instance &operator=(Instance &&other) noexcept;

  bool owns() const noexcept;

private:
  void swap(Instance &other) noexcept;

  bool _owns{false};
};
} // namespace glfw
} // namespace marlon

#endif