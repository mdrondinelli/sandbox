#include "particle.h"

namespace marlon {
namespace physics {
void Particle_construction_queue::push(
    Particle_create_info const &create_info) {
  create_infos.push_back(create_info);
};

std::span<Particle_create_info const>
Particle_construction_queue::get() const noexcept {
  return create_infos;
}

void Particle_destruction_queue::push(Particle_reference reference) {
  references.push_back(reference);
}

std::span<Particle_reference const>
Particle_destruction_queue::get() const noexcept {
  return references;
}
} // namespace physics
} // namespace marlon