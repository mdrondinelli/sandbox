#ifndef MARLON_ENGINE_APP_H
#define MARLON_ENGINE_APP_H

#include <string>
#include <string_view>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "camera.h"

namespace marlon {
namespace engine {
struct App_create_info {
  physics::World_create_info physics_world_create_info;
  physics::World_simulate_info physics_world_simulate_info;
  math::Vec2i default_window_extents{1280, 720};
  std::string_view default_window_title{"app"};
  Camera_create_info camera_create_info;
};

class App {
public:
  explicit App(App_create_info const &create_info);

  virtual ~App();

  int run();

protected:
  physics::World *get_world() noexcept;

  graphics::Graphics *get_graphics() noexcept;

  graphics::Scene *get_scene() noexcept;

  Camera *get_camera() noexcept;

  virtual void pre_loop() {}

  virtual void post_loop() {}

  virtual void pre_physics() {}

  virtual void post_physics() {}

private:
  class Runtime;

  void loop();

  physics::World_create_info _physics_world_create_info;
  physics::World_simulate_info _physics_world_simulate_info;
  math::Vec2i _default_window_extents;
  std::string _default_window_title;
  Camera_create_info _camera_create_info;
  std::unique_ptr<Runtime> _runtime;
};
} // namespace engine
} // namespace marlon

#endif