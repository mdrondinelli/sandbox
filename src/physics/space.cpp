#include "space.h"

#include <unordered_map>

namespace marlon {
namespace physics {
namespace {
struct Particle {
  math::Vec3f position;
  math::Vec3f velocity;
  float mass;
};
} // namespace

struct Space_state::Impl {
  std::uint64_t next_particle_reference_value{};
  std::unordered_map<Particle_reference, Particle> particles;
};

Space_state::Space_state() : _impl{std::make_unique<Impl>()} {}

Space_state::~Space_state() {}

Space_state Space_state::step(Space_state_step_info const &step_info) {
  Space_state retval;
  retval._impl->particles = _impl->particles;
  for (auto const reference : step_info.destroyed_particles) {
    retval._impl->particles.erase(reference);
  }
  for (auto &[reference, particle] : retval._impl->particles) {
    particle.position += particle.velocity * step_info.delta_time;
    step_info.particle_motion_callback(
        {reference, particle.position, particle.velocity});
  }
  for (auto const &create_info : step_info.created_particles) {
    Particle_reference const reference{
        retval._impl->next_particle_reference_value};
    Particle const particle{create_info.position, create_info.velocity,
                            create_info.mass};
    retval._impl->particles.emplace(reference, particle);
    ++retval._impl->next_particle_reference_value;
    *create_info.out_reference = reference;
  }
  return retval;
}
} // namespace physics
} // namespace marlon