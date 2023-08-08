#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>

#include "particle.h"
#include "static_rigid_body.h"

namespace marlon {
namespace physics {
struct Space_create_info {
  math::Vec3f gravitational_acceleration{math::Vec3f::zero()};
};

struct Space_simulate_info {
  float delta_time;
  int substep_count;
};

class Space {
public:
  explicit Space(Space_create_info const &create_info);

  ~Space();

  Particle_reference create_particle(Particle_create_info const &create_info);

  void destroy_particle(Particle_reference particle);

  Static_rigid_body_reference
  create_static_rigid_body(Static_rigid_body_create_info const &create_info);

  void destroy_static_rigid_body(Static_rigid_body_reference static_rigid_body);

  void simulate(Space_simulate_info const &simulate_info);

private:
  class Impl;

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif