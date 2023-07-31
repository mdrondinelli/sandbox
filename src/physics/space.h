#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>
#include <span>

#include "../math/vec.h"
#include "particle.h"
#include "static_rigid_body.h"

namespace marlon {
namespace physics {
struct Space_simulate_info {
  float delta_time;
  int substep_count;
};

class Space {
public:
  Particle_reference create_particle(Particle_create_info const &create_info);

  void destroy_particle(Particle_reference particle);

  Static_rigid_body_reference
  create_static_rigid_body(Static_rigid_body_create_info const &create_info);

  void destroy_static_rigid_body(Static_rigid_body_reference static_rigid_body);

  void simulate(Space_simulate_info const &simulate_info);

private:
  struct Particle {
    std::uint64_t collision_flags;
    std::uint64_t collision_mask;
    math::Vec3f previous_position;
    math::Vec3f current_position;
    math::Vec3f velocity;
    math::Vec3f acceleration;
    float damping_factor;
    float inverse_mass;
    float radius;
    Particle_motion_callback *motion_callback;
  };

  struct Static_rigid_body {
    std::uint64_t collision_flags;
    std::uint64_t collision_mask;
    math::Mat3x4f transform;
    math::Mat3x4f transform_inverse;
    Shape *shape;
  };

  void flatten_particles();

  void flatten_static_rigid_bodies();

  void find_particle_particle_collisions();

  void find_particle_static_rigid_body_collisions();

  void solve_particle_particle_collisions();

  void solve_particle_static_rigid_body_collisions();

  std::unordered_map<Particle_reference, Particle> _particles;
  std::unordered_map<Static_rigid_body_reference, Static_rigid_body>
      _static_rigid_bodies;
  std::vector<Particle *> _flattened_particles;
  std::vector<Static_rigid_body *> _flattened_static_rigid_bodies;
  std::vector<std::pair<Particle *, Particle *>> _particle_particle_collisions;
  std::vector<std::pair<Particle *, Static_rigid_body *>>
      _particle_static_rigid_body_collisions;
  std::uint64_t _next_particle_reference_value{};
  std::uint64_t _next_static_rigid_body_reference_value{};
};
} // namespace physics
} // namespace marlon

#endif