#ifndef MARLON_ENGINE_APP_H
#define MARLON_ENGINE_APP_H

#include <string>
#include <string_view>

#include "graphics/camera.h"
#include "graphics/graphics.h"
#include "graphics/render_stream.h"
#include "graphics/scene.h"
#include "window.h"

namespace marlon {
namespace engine {
struct App_create_info {
  std::string_view window_title{"app"};
  math::Vec2i window_extents{1280, 720};
  bool full_screen{false};
};

class App {
public:
  explicit App(App_create_info const &create_info);

  virtual ~App();

  /** only call once per App lifetime */
  int run();

protected:
  Window const *get_window() const noexcept;

  Window *get_window() noexcept;

  graphics::Graphics const *get_graphics() const noexcept;

  graphics::Graphics *get_graphics() noexcept;

  graphics::Render_stream *create_render_stream(graphics::Scene const *scene,
                                                graphics::Camera const *camera);

  graphics::Unique_render_stream
  create_render_stream_unique(graphics::Scene const *scene,
                              graphics::Camera const *camera);

  bool is_looping() const noexcept;

  void stop_looping() noexcept;

  double get_loop_iteration_wall_time() const noexcept;

  virtual void pre_loop();

  virtual void pre_input() {}

  virtual void post_input() {}

  virtual void pre_render() {}

  virtual void post_render() {}

private:
  class Runtime;

  void do_loop();

  void do_input();

  std::string _window_title;
  math::Vec2i _window_extents;
  bool _full_screen;
  std::unique_ptr<Runtime> _runtime;
  bool _looping{false};
  double _loop_iteration_wall_time{0.0};
};
} // namespace engine
} // namespace marlon

#endif
