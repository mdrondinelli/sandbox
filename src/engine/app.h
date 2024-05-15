#ifndef MARLON_ENGINE_APP_H
#define MARLON_ENGINE_APP_H

#include <string>
#include <string_view>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "../util/thread_pool.h"
#include "window.h"

namespace marlon {
namespace engine {
struct App_create_info {
  physics::World_create_info world_create_info;
  physics::World_simulate_info world_simulate_info;
  math::Vec2i window_extents{1280, 720};
  std::string_view window_title{"app"};
};

class App {
public:
  explicit App(App_create_info const &create_info);

  virtual ~App();

  int run();

protected:
  physics::World const *get_world() const noexcept;

  physics::World *get_world() noexcept;

  physics::World_simulate_result const &
  get_world_simulate_result() const noexcept;

  Window const *get_window() const noexcept;

  Window *get_window() noexcept;

  graphics::Graphics const *get_graphics() const noexcept;

  graphics::Graphics *get_graphics() noexcept;

  graphics::Scene const *get_scene() const noexcept;

  graphics::Scene *get_scene() noexcept;

  graphics::Camera const *get_camera() const noexcept;

  graphics::Camera *get_camera() noexcept;

  bool is_looping() const noexcept;

  void stop_looping() noexcept;

  double get_loop_iteration_wall_time() const noexcept;

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
  physics::World_simulate_result _world_simulate_result;
  math::Vec2i _window_extents;
  std::string _window_title;
  std::unique_ptr<Runtime> _runtime;
  bool _looping{false};
  double _loop_iteration_wall_time{0.0};
};
} // namespace engine
} // namespace marlon

#endif