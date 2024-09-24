#include "game.h"

namespace marlon::game {
using namespace physics;
using namespace math;
Game::Game(Game_create_info const &)
    : _world{
          {
              .max_particles = 0,
              .max_rigid_bodies = max_rigid_bodies,
              .max_static_bodies = max_static_bodies,
              .max_aabb_tree_leaf_nodes = max_boxes,
              .max_aabb_tree_internal_nodes = max_boxes,
              .max_neighbor_pairs = max_boxes,
              .max_neighbor_groups = max_boxes,
              .gravitational_acceleration = gravitational_acceleration,
          },
      }, _timeouts{{
          .max_timeouts = 1000,
            
      }} {}

void Game::tick() {
  _world.simulate({World_simulate_info{tick_duration, physics_substep_count}});
}
} // namespace marlon::game
