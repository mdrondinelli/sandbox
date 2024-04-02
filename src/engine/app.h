#ifndef MARLON_ENGINE_APP_H
#define MARLON_ENGINE_APP_H

#include <string>
#include <string_view>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "camera.h"
#include "window.h"

namespace marlon {
namespace engine {
struct App_create_info {
  physics::World_create_info world_create_info;
  physics::World_simulate_info world_simulate_info;
  math::Vec2i window_extents{1280, 720};
  std::string_view window_title{"app"};
  Camera_create_info camera_create_info;
};

class App {
public:
  explicit App(App_create_info const &create_info);

  virtual ~App();

  int run();

protected:
  physics::World *get_world() noexcept;

  Window *get_window() noexcept;

  Camera *get_camera() noexcept;

  graphics::Graphics *get_graphics() noexcept;

  graphics::Scene *get_scene() noexcept;

  double get_delta_time() const noexcept;

  virtual void pre_loop() {}

  virtual void post_loop() {}

  virtual void pre_input() {}

  virtual void post_input() {}

  virtual void pre_physics() {}

  virtual void post_physics() {}

private:
  class Runtime;

  void loop();

  physics::World_create_info _world_create_info;
  physics::World_simulate_info _world_simulate_info;
  math::Vec2i _window_extents;
  std::string _window_title;
  Camera_create_info _camera_create_info;
  std::unique_ptr<Runtime> _runtime;
  double _delta_time;
};
} // namespace engine
} // namespace marlon

#endif