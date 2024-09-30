#include <game/game.h>
#include <graphics/camera.h>
#include <graphics/scene.h>
#include <platform/app.h>

using namespace marlon;

class Game_app : public platform::App {
public:
  Game_app()
      : platform::App{{
            .window_extents = {1280, 720},
            .full_screen = false,
        }} {}

  ~Game_app() {
    get_graphics()->destroy_render_stream(_render_stream);
  }

  void post_input() override {
    if (get_window()->should_close()) {
      stop_looping();
    } else {
      _accumulated_time += get_loop_iteration_wall_time();
      while (_accumulated_time >= game::Game::tick_duration) {
        _accumulated_time -= game::Game::tick_duration;
        _game.tick();
      }
      _render_stream->render();
    }
  }

private:
  game::Game _game{{}};
  graphics::Scene _scene{{}};
  graphics::Camera _camera{};
  graphics::Render_stream *_render_stream{
      create_render_stream(&_scene, &_camera)};
  double _accumulated_time{};
};

int main() {
  Game_app{}.run();
  return 0;
}
