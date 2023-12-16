#include "application_loop.h"

#include <cassert>

namespace marlon {
namespace client {
Application_loop::Application_loop(
    Application_loop_create_info const &create_info)
    : _space{create_info.space},
      _physics_step_duration{create_info.physics_step_duration},
      _physics_substep_count{create_info.physics_substep_count},
      _max_physics_island_position_iterations{
          create_info.max_physics_island_position_iterations},
      _max_physics_island_velocity_iterations{
          create_info.max_physics_island_velocity_iterations},
      _accumulated_time{0.0} {
  assert(create_info.space != nullptr);
  assert(create_info.physics_step_duration > 0.0f);
  assert(create_info.physics_substep_count >= 1);
}

bool Application_loop::run_once(double delta_time) {
  auto retval = false;
  _accumulated_time += delta_time;
  while (_accumulated_time >= _physics_step_duration) {
    _accumulated_time -= _physics_step_duration;
    _space->simulate({.delta_time = _physics_step_duration,
                      .substep_count = _physics_substep_count,
                      .max_island_position_iterations =
                          _max_physics_island_position_iterations,
                      .max_island_velocity_iterations =
                          _max_physics_island_velocity_iterations});
    retval = true;
  }
  return retval;
}
} // namespace client
} // namespace marlon