#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>

#include "dynamic_rigid_body.h"
#include "particle.h"
#include "static_rigid_body.h"

namespace marlon {
namespace physics {
struct Space_create_info {
  std::ptrdiff_t max_particles{1000000};
  std::ptrdiff_t max_static_rigid_bodies{1000000};
  math::Vec3f gravitational_acceleration{math::Vec3f::zero()};
};

struct Space_simulate_info {
  float delta_time;
  int substep_count{1};
};

class Space {
public:
  explicit Space(Space_create_info const &create_info);

  ~Space();

  Particle_handle create_particle(Particle_create_info const &create_info);

  void destroy_particle(Particle_handle handle);

  Static_rigid_body_handle
  create_static_rigid_body(Static_rigid_body_create_info const &create_info);

  void destroy_static_rigid_body(Static_rigid_body_handle handle);

  Dynamic_rigid_body_handle
  create_dynamic_rigid_body(Dynamic_rigid_body_create_info const &create_info);

  void destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle);

  void simulate(Space_simulate_info const &simulate_info);

private:
  class Impl;

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif