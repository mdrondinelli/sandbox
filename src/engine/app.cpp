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
#include "glfw_init_guard.h"
#include "window.h"

namespace marlon {
using namespace math;
using util::Scheduling_policy;
using util::Thread_pool;

namespace engine {
class App::Runtime {
public:
  explicit Runtime(physics::World_create_info const &world_create_info,
                   Window_create_info const &window_create_info)
      : // _threads{std::max(
        //       std::max(std::thread::hardware_concurrency() / 2, 1u) - 1,
        //       1u)},
        _world{world_create_info},
        _window{window_create_info},
        _graphics{[&]() {
          glfwMakeContextCurrent(_window.get_glfw_window());
          glfwSwapInterval(0);
          return graphics::gl::Graphics{{
              .loader = glfwGetProcAddress,
              .window = &_window,
          }};
        }()},
        _scene{_graphics.create_scene_unique({})},
        _render_stream{_graphics.create_render_stream({
            .target = _graphics.get_default_render_target(),
            .scene = _scene.get(),
            .camera = &_camera,
        })} {}

  // Thread_pool *get_threads() noexcept { return &_threads; }

  physics::World *get_world() noexcept { return &_world; }

  Window *get_window() noexcept { return &_window; }

  graphics::Graphics *get_graphics() noexcept { return &_graphics; }

  graphics::Render_target *get_render_target() noexcept {
    return _graphics.get_default_render_target();
  }

  graphics::Scene *get_scene() noexcept { return _scene.get(); }

  graphics::Camera *get_camera() noexcept { return &_camera; }

  void render() {
    _render_stream->render();
    glfwSwapBuffers(_window.get_glfw_window());
  }

private:
  // Thread_pool _threads;
  physics::World _world;
  Glfw_init_guard _glfw_init_guard;
  Window _window;
  graphics::gl::Graphics _graphics;
  graphics::Unique_scene _scene;
  graphics::Camera _camera;
  graphics::Unique_render_stream _render_stream;
};

App::App(App_create_info const &create_info)
    : _world_create_info{create_info.world_create_info},
      _world_simulate_info{create_info.world_simulate_info},
      _window_extents{create_info.window_extents},
      _window_title{create_info.window_title} {}

App::~App() {}

int App::run() {
  assert(_runtime == nullptr);
  auto result = 0;
  // try {
  _runtime = std::make_unique<Runtime>(_world_create_info,
                                       Window_create_info{
                                           .extents = _window_extents,
                                           .title = _window_title.c_str(),
                                       });
  pre_loop();
  loop();
  post_loop();
  // } catch (std::exception &e) {
  //   std::cerr << "Caught exception: " << e.what() << "\n";
  //   result = 1;
  // } catch (...) {
  //   std::cerr << "Caught unknown exception\n";
  //   result = 2;
  // }
  _runtime = nullptr;
  return result;
}

physics::World const *App::get_world() const noexcept {
  return _runtime->get_world();
}

physics::World *App::get_world() noexcept { return _runtime->get_world(); }

physics::World_simulate_result const &
App::get_world_simulate_result() const noexcept {
  return _world_simulate_result;
}

Window const *App::get_window() const noexcept {
  return _runtime->get_window();
}

Window *App::get_window() noexcept { return _runtime->get_window(); }

graphics::Graphics const *App::get_graphics() const noexcept {
  return _runtime->get_graphics();
}

graphics::Graphics *App::get_graphics() noexcept {
  return _runtime->get_graphics();
}

graphics::Scene const *App::get_scene() const noexcept {
  return _runtime->get_scene();
}

graphics::Scene *App::get_scene() noexcept { return _runtime->get_scene(); }

graphics::Camera const *App::get_camera() const noexcept {
  return _runtime->get_camera();
}

graphics::Camera *App::get_camera() noexcept { return _runtime->get_camera(); }

bool App::is_looping() const noexcept { return _looping; }

void App::stop_looping() noexcept { _looping = false; }

double App::get_loop_iteration_wall_time() const noexcept {
  return _loop_iteration_wall_time;
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
      _world_simulate_result =
          _runtime->get_world()->simulate(_world_simulate_info);
      post_physics();
    }
    _runtime->render();
  }
}
} // namespace engine
} // namespace marlon