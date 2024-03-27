#include "application_loop.h"

#include <cassert>

namespace marlon {
namespace client {
Application_loop::Application_loop(
    Application_loop_create_info const &create_info)
    : _space{create_info.space},
      _physics_step_duration{create_info.physics_step_duration},
      _physics_substep_count{create_info.physics_substep_count},
      _min_position_iterations_per_contact{
          create_info.min_position_iterations_per_contact},
      _max_position_iterations_per_contact{
          create_info.max_position_iterations_per_contact},
      _min_velocity_iterations_per_contact{
          create_info.min_velocity_iterations_per_contact},
      _max_velocity_iterations_per_contact{
          create_info.max_velocity_iterations_per_contact},
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
                      .min_desired_position_iterations_per_contact =
                          _min_position_iterations_per_contact,
                      .max_desired_position_iterations_per_contact =
                          _max_position_iterations_per_contact,
                      .min_desired_velocity_iterations_per_contact =
                          _min_velocity_iterations_per_contact,
                      .max_desired_velocity_iterations_per_contact =
                          _max_velocity_iterations_per_contact});
    retval = true;
  }
  return retval;
}
} // namespace client
} // namespace marlon