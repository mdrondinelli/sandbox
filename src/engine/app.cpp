#include "app.h"

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
    : _physics_world_create_info{create_info.physics_world_create_info},
      _physics_world_simulate_info{create_info.physics_world_simulate_info},
      _default_window_extents{create_info.default_window_extents},
      _default_window_title{create_info.default_window_title},
      _camera_create_info{create_info.camera_create_info} {}

App::~App() {}

int App::run() {
  assert(_runtime == nullptr);
  auto result = 0;
  try {
    _runtime =
        std::make_unique<Runtime>(_physics_world_create_info,
                                  Window_create_info{
                                      .extents = _default_window_extents,
                                      .title = _default_window_title.c_str(),
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

graphics::Graphics *App::get_graphics() noexcept {
  return _runtime->get_graphics();
}

graphics::Scene *App::get_scene() noexcept { return _runtime->get_scene(); }

Camera *App::get_camera() noexcept { return _runtime->get_camera(); }

void App::loop() {
  auto previous_time = glfwGetTime();
  auto accumulated_time = 0.0;
  for (;;) {
    glfwPollEvents();
    if (glfwWindowShouldClose(_runtime->get_window()->get_glfw_window())) {
      break;
    }
    auto const current_time = glfwGetTime();
    auto const elapsed_time = current_time - previous_time;
    previous_time = current_time;
    accumulated_time += elapsed_time;
    auto render_required = false;
    while (accumulated_time > _physics_world_simulate_info.delta_time) {
      accumulated_time -= _physics_world_simulate_info.delta_time;
      render_required = true;
      pre_physics();
      _runtime->get_world()->simulate(_physics_world_simulate_info);
      post_physics();
    }
    if (render_required) {
      _runtime->render();
    }
  }
}
} // namespace engine
} // namespace marlon