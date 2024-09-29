#include <game/game.h>
#include <platform/app.h>

using namespace marlon;
class App : public platform::App {
public:
  App()
      : platform::App{{
            .window_extents = {1280, 720},
            .full_screen = false,
        }} {}

  void post_input() override {
    if (get_window()->should_close()) {
      stop_looping();
    } else {
      _accumulated_time += get_loop_iteration_wall_time();
      while (_accumulated_time >= game::Game::tick_duration) {
        _accumulated_time -= game::Game::tick_duration;
        _game.tick();
      }
    }
  }

private:
  game::Game _game{{}};
  double _accumulated_time{0.0};
};

int main() {
  App{}.run();
  return 0;
}
