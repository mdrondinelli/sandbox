#ifndef MARLON_CLIENT_APPLICATION_LOOP_H
#define MARLON_CLIENT_APPLICATION_LOOP_H

#include "../physics/physics.h"

namespace marlon {
namespace client {
struct Application_loop_create_info {
  physics::Space *space{};
  float physics_step_duration{1.0f / 60.0f};
  int physics_substep_count{16};
  int max_physics_island_position_iterations{32};
  int max_physics_island_velocity_iterations{32};
};

class Application_loop {
public:
  explicit Application_loop(
      Application_loop_create_info const &create_info);

  bool run_once(double delta_time);

private:
  physics::Space *_space;
  float _physics_step_duration;
  int _physics_substep_count;
  int _max_physics_island_position_iterations;
  int _max_physics_island_velocity_iterations;
  double _accumulated_time;
};
} // namespace client
} // namespace marlon

#endif