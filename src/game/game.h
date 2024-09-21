#ifndef MARLON_GAME_GAME_H
#define MARLON_GAME_GAME_H

#include "game/box.h"
#include "game/game_state.h"
#include "game/timeout.h"
#include "physics/world.h"

namespace marlon::game {
class Timeout_callback;

struct Game_create_info {};

class Game {
public:
  static constexpr auto tick_duration = 1.0 / 120.0;
  static constexpr auto physics_substep_count = 10;
  static constexpr auto max_rigid_bodies = 10000;
  static constexpr auto max_static_bodies = 100;
  static constexpr auto max_boxes = 10000;
  static constexpr auto gravitational_acceleration = -9.81f * math::Vec3f::y_axis();

  Game() = default;

  explicit Game(Game_create_info const &);

  void tick();

  Timeout_manager const &get_timeouts() const noexcept;

  Timeout_manager &get_timeouts() noexcept;

  Box_manager const &get_boxes() const noexcept;

  Box_manager &get_boxes() noexcept;

private:
  physics::World _world;
  Timeout_manager _timeouts;
  Box_manager _boxes;
  Columns_game_state _game_state;
};
} // namespace marlon::game

#endif
