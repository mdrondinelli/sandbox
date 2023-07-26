#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>
#include <span>

#include "../math/vec.h"
#include "particle.h"

namespace marlon {
namespace physics {
struct Space_simulate_info {
  math::Vec3f acceleration;
  float delta_time;
  int substep_count;
};

class Space {
public:
  Particle_reference create_particle(Particle_create_info const &create_info);

  void destroy_particle(Particle_reference particle);

  void simulate(Space_simulate_info const &simulate_info);

  // void step(Space_step_info const &step_info);

private:
  struct Particle {
    math::Vec3f previous_position;
    math::Vec3f current_position;
    math::Vec3f velocity;
    float inverse_mass;
    float radius;
    Particle_motion_callback *motion_callback;
  };

  std::vector<std::pair<Particle *, Particle *>> find_particle_collisions();
  void solve_particle_collisions(
      std::span<std::pair<Particle *, Particle *> const> particle_collisions);

  std::uint64_t _next_particle_reference_value{};
  std::unordered_map<Particle_reference, Particle> _particles;
};
} // namespace physics
} // namespace marlon

#endif