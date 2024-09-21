#include "app.h"

#include <chrono>

#include <glad/gl.h>

#include <GLFW/glfw3.h>

#include <graphics/gl/graphics.h>

#include "glfw_init_guard.h"
#include "window.h"

namespace marlon::engine {
class App::Runtime {
public:
  explicit Runtime(Window_create_info const &window_create_info)
      : _window{window_create_info},
        _graphics{[&]() {
          return graphics::gl::Graphics{{
              .loader = glfwGetProcAddress,
              .window = &_window,
          }};
        }()},
        _scene{{}},
        _render_stream{_graphics.create_render_stream({
            .target = _graphics.get_default_render_target(),
            .scene = &_scene,
            .camera = &_camera,
        })} {
    glfwSwapInterval(0);
  }

  Window *get_window() noexcept {
    return &_window;
  }

  graphics::gl::Graphics *get_graphics() noexcept {
    return &_graphics;
  }

  graphics::Scene *get_scene() noexcept {
    return &_scene;
  }

  graphics::Camera *get_camera() noexcept {
    return &_camera;
  }

  void render() {
    _render_stream->render();
  }

private:
  Glfw_init_guard _glfw_init_guard;
  Window _window;
  graphics::gl::Graphics _graphics;
  graphics::Scene _scene;
  graphics::Camera _camera;
  graphics::Unique_render_stream _render_stream;
};

App::App(App_create_info const &create_info)
    : _window_title{create_info.window_title},
      _window_extents{create_info.window_extents},
      _full_screen{create_info.full_screen} {}

App::~App() {}

int App::run() {
  assert(_runtime == nullptr);
  auto result = 0;
  // try {
  _runtime = std::make_unique<Runtime>(Window_create_info{
      .title = _window_title.c_str(),
      .extents = _window_extents,
      .full_screen = _full_screen,
  });
  pre_loop();
  do_loop();
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

Window const *App::get_window() const noexcept {
  return _runtime->get_window();
}

Window *App::get_window() noexcept {
  return _runtime->get_window();
}

graphics::Graphics const *App::get_graphics() const noexcept {
  return _runtime->get_graphics();
}

graphics::Graphics *App::get_graphics() noexcept {
  return _runtime->get_graphics();
}

graphics::Render_stream *App::create_render_stream(graphics::Scene const *scene, graphics::Camera const *camera) {
  auto const graphics = _runtime->get_graphics();
  return graphics->create_render_stream(
      {.target = graphics->get_default_render_target(), .scene = scene, .camera = camera});
}

graphics::Unique_render_stream App::create_render_stream_unique(graphics::Scene const *scene,
                                                                graphics::Camera const *camera) {
  auto const graphics = _runtime->get_graphics();
  return graphics->create_render_stream_unique(
      {.target = graphics->get_default_render_target(), .scene = scene, .camera = camera});
}

bool App::is_looping() const noexcept {
  return _looping;
}

void App::stop_looping() noexcept {
  _looping = false;
}

double App::get_loop_iteration_wall_time() const noexcept {
  return _loop_iteration_wall_time;
}

void App::do_loop() {
  using clock = std::chrono::high_resolution_clock;
  using duration = std::chrono::duration<double>;
  _loop_iteration_wall_time = 0.0;
  auto previous_time = clock::now();
  _looping = true;
  while (_looping) {
    auto const current_time = clock::now();
    _loop_iteration_wall_time = std::chrono::duration_cast<duration>(current_time - previous_time).count();
    previous_time = current_time;
    do_input();
  }
}

void App::do_input() {
  _runtime->get_window()->pre_input();
  pre_input();
  glfwPollEvents();
  post_input();
}
} // namespace marlon::engine
