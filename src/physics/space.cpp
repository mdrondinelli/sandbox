#include "space.h"

#include <unordered_map>

namespace marlon {
namespace physics {
Particle_reference
Space::create_particle(Particle_create_info const &create_info) {
  Particle_reference const reference{_next_particle_reference_value};
  Particle const particle{create_info.position, create_info.velocity,
                          create_info.mass, create_info.motion_callback};
  _particles.emplace(reference, particle);
  ++_next_particle_reference_value;
  return reference;
}

void Space::destroy_particle(Particle_reference particle) {
  _particles.erase(particle);
}

void Space::simulate(Space_simulate_info const &simulate_info) {
  for (auto &[reference, value] : _particles) {
    value.velocity += simulate_info.acceleration * simulate_info.delta_time;
    value.position += value.velocity * simulate_info.delta_time;
    if (value.motion_callback != nullptr) {
      value.motion_callback->on_particle_motion(
          {reference, value.position, value.velocity});
    }
  }
}
} // namespace physics
} // namespace marlon