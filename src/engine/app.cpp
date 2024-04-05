#include "app.h"

#include <chrono>
#include <iostream>
#include <stdexcept>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#endif

#include <GLFW/glfw3.h>

#include "../graphics/gl/graphics.h"
#include "camera.h"
#include "glfw_init_guard.h"
#include "window.h"

namespace marlon {
using namespace math;

namespace engine {
class App::Runtime {
public:
  explicit Runtime(physics::World_create_info const &world_create_info,
                   Window_create_info const &window_create_info,
                   Camera_create_info const &camera_create_info)
      : _world{world_create_info},
        _window{window_create_info},
        _camera{camera_create_info},
        _graphics{[&]() {
          glfwMakeContextCurrent(_window.get_glfw_window());
          glfwSwapInterval(0);
          return graphics::Gl_graphics{{
              .function_loader = glfwGetProcAddress,
              .window = &_window,
          }};
        }()},
        _scene{_graphics.create_scene_unique({})} {}

  physics::World *get_world() noexcept { return &_world; }

  Window *get_window() noexcept { return &_window; }

  Camera *get_camera() noexcept { return &_camera; }

  graphics::Graphics *get_graphics() noexcept { return &_graphics; }

  graphics::Render_target *get_render_target() noexcept {
    return _graphics.get_default_render_target();
  }

  graphics::Scene *get_scene() noexcept { return _scene.get(); }

  void render() {
    _graphics.render({
        .target = _graphics.get_default_render_target(),
        .source = _scene.get(),
        .position = _camera.get_position(),
        .orientation = _camera.get_orientation(),
        .zoom = _camera.get_zoom(),
        .near_plane_distance = _camera.get_near_plane_distance(),
        .far_plane_distance = _camera.get_far_plane_distance(),
    });
    glfwSwapBuffers(_window.get_glfw_window());
  }

private:
  physics::World _world;
  Glfw_init_guard _glfw_init_guard;
  Window _window;
  Camera _camera;
  graphics::Gl_graphics _graphics;
  graphics::Unique_scene_ptr _scene;
};

App::App(App_create_info const &create_info)
    : _world_create_info{create_info.world_create_info},
      _world_simulate_info{create_info.world_simulate_info},
      _window_extents{create_info.window_extents},
      _window_title{create_info.window_title},
      _camera_create_info{create_info.camera_create_info} {}

App::~App() {}

int App::run() {
  assert(_runtime == nullptr);
  auto result = 0;
  try {
    _runtime = std::make_unique<Runtime>(_world_create_info,
                                         Window_create_info{
                                             .extents = _window_extents,
                                             .title = _window_title.c_str(),
                                         },
                                         _camera_create_info);
    pre_loop();
    loop();
    post_loop();
  } catch (std::exception &e) {
    std::cerr << "Caught exception: " << e.what() << "\n";
    result = 1;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    result = 2;
  }
  _runtime = nullptr;
  return result;
}

physics::World *App::get_world() noexcept { return _runtime->get_world(); }

Window *App::get_window() noexcept { return _runtime->get_window(); }

Camera *App::get_camera() noexcept { return _runtime->get_camera(); }

graphics::Graphics *App::get_graphics() noexcept {
  return _runtime->get_graphics();
}

graphics::Scene *App::get_scene() noexcept { return _runtime->get_scene(); }

bool App::is_looping() const noexcept { return _looping; }

void App::stop_looping() noexcept { _looping = false; }

double App::get_loop_iteration_wall_time() const noexcept {
  return _loop_iteration_wall_time;
}

double App::get_physics_simulation_wall_time() const noexcept {
  return _physics_simulation_wall_time;
}

void App::loop() {
  using clock = std::chrono::high_resolution_clock;
  using duration = std::chrono::duration<double>;
  _loop_iteration_wall_time = 0.0;
  auto previous_time = clock::now();
  auto accumulated_time = 0.0;
  _looping = true;
  while (_looping) {
    auto const current_time = clock::now();
    _loop_iteration_wall_time =
        std::chrono::duration_cast<duration>(current_time - previous_time)
            .count();
    previous_time = current_time;
    accumulated_time += _loop_iteration_wall_time;
    _runtime->get_window()->pre_input();
    pre_input();
    glfwPollEvents();
    post_input();
    if (accumulated_time >= _world_simulate_info.delta_time) {
      accumulated_time -= _world_simulate_info.delta_time;
      pre_physics();
      auto const physics_begin_time = clock::now();
      _runtime->get_world()->simulate(_world_simulate_info);
      auto const physics_end_time = clock::now();
      _physics_simulation_wall_time = std::chrono::duration_cast<duration>(
                                          physics_end_time - physics_begin_time)
                                          .count();
      post_physics();
    }
    _runtime->render();
  }
}
} // namespace engine
} // namespace marlon