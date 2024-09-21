#ifndef MARLON_GAME_GAME_STATE_H
#define MARLON_GAME_GAME_STATE_H

#include "timeout.h"

namespace marlon::game {
class Game;

class Game_state {
public:
  virtual ~Game_state() = default;

  virtual void initialize() = 0;
};

class Columns_game_state : public Game_state, private Timeout_callback {
public:
  Columns_game_state() = default;

  explicit Columns_game_state(Game *game)
      : _game{game} {}

  void initialize() final;

private:
  void on_timeout() final;

  Game *_game{};
};
} // namespace marlon::game

#endif
