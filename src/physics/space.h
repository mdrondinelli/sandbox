#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_STATE_H

#include <functional>
#include <memory>
#include <span>

#include "particle.h"

namespace marlon {
namespace physics {
struct Space_state_step_info {
  std::function<void(Particle_motion_event const &)> particle_motion_callback;
  std::span<Particle_create_info const> created_particles;
  std::span<Particle_reference const> destroyed_particles;
  float delta_time;
  int substep_count;
};

class Space_state {
public:
  Space_state();

  ~Space_state();

  Space_state(Space_state &&other) noexcept : _impl{std::move(other._impl)} {}

  Space_state &operator=(Space_state &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  // semantics:
  // destroyed particles removed
  // simulation step + callbacks
  // created particles added
  Space_state step(Space_state_step_info const &step_info);

private:
  struct Impl;

  void swap(Space_state &other) { _impl.swap(other._impl); }

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif