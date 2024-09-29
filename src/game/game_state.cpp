#include "game_state.h"

#include <iostream>

#include "game.h"

namespace marlon::game {
void Columns_game_state::initialize() {
  _game->get_timeouts().set_interval(this, 0.5);
}

void Columns_game_state::on_timeout() {
  std::cout << "Kick!\a\n";
}
} // namespace marlon::game
