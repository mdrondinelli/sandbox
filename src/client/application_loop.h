#ifndef MARLON_CLIENT_APPLICATION_LOOP_H
#define MARLON_CLIENT_APPLICATION_LOOP_H

#include "../physics/physics.h"

namespace marlon {
namespace client {
struct Application_loop_create_info {
  physics::Space *space{};
  float tick_duration{1.0f / 30.0f};
  int physics_substep_count{4};
};

class Application_loop {
public:
  explicit Application_loop(
      Application_loop_create_info const &create_info);

  bool run_once(double delta_time);

private:
  physics::Space *_space;
  float _tick_duration;
  int _physics_substep_count;
  double _accumulated_time;
};
} // namespace client
} // namespace marlon

#endif