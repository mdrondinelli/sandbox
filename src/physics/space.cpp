#include "space.h"

#include <unordered_map>

namespace marlon {
namespace physics {
Particle_reference
Space::create_particle(Particle_create_info const &create_info) {
  Particle_reference const reference{_next_particle_reference_value};
  Particle const particle{.previous_position = create_info.position,
                          .current_position = create_info.position,
                          .velocity = create_info.velocity,
                          .inverse_mass = 1.0f / create_info.mass,
                          .radius = create_info.radius,
                          .motion_callback = create_info.motion_callback};
  _particles.emplace(reference, particle);
  ++_next_particle_reference_value;
  return reference;
}

void Space::destroy_particle(Particle_reference particle) {
  _particles.erase(particle);
}

void Space::simulate(Space_simulate_info const &simulate_info) {
  auto const h = simulate_info.delta_time / simulate_info.substep_count;
  auto const h_inv = 1.0f / h;
  auto const particle_collisions = find_particle_collisions();
  for (auto i = 0; i < simulate_info.substep_count; ++i) {
    for (auto &[reference, value] : _particles) {
      value.previous_position = value.current_position;
      value.velocity += h * simulate_info.acceleration;
      value.current_position += h * value.velocity;
    }
    solve_particle_collisions(particle_collisions);
    for (auto &[reference, value] : _particles) {
      value.velocity =
          h_inv * (value.current_position - value.previous_position);
    }
  }
  for (auto &[reference, value] : _particles) {
    if (value.motion_callback != nullptr) {
      value.motion_callback->on_particle_motion(
          {reference, value.current_position, value.velocity});
    }
  }
}

std::vector<std::pair<Space::Particle *, Space::Particle *>>
Space::find_particle_collisions() {
  std::vector<Particle *> particles;
  particles.reserve(_particles.size());
  for (auto &pair : _particles) {
    particles.emplace_back(&pair.second);
  }
  std::vector<std::pair<Particle *, Particle *>> particle_collisions;
  for (std::size_t i{}; i + 1 < particles.size(); ++i) {
    for (std::size_t j{i + 1}; j < particles.size(); ++j) {
      auto const particle_a = particles[i];
      auto const particle_b = particles[j];
      auto const displacement =
          particle_b->current_position - particle_a->current_position;
      auto const distance2 = math::length2(displacement);
      auto const contact_distance = particle_a->radius + particle_b->radius;
      auto const contact_distance2 = contact_distance * contact_distance;
      if (distance2 < contact_distance2) {
        particle_collisions.emplace_back(particle_a, particle_b);
      }
    }
  }
  return particle_collisions;
}

void Space::solve_particle_collisions(
    std::span<std::pair<Particle *, Particle *> const> particle_collisions) {
  for (auto const [particle_a, particle_b] : particle_collisions) {
    auto const displacement =
        particle_b->current_position - particle_a->current_position;
    auto const distance2 = math::length2(displacement);
    auto const contact_distance = particle_a->radius + particle_b->radius;
    if (distance2 == 0.0f) {
      // particles coincide, pick arbitrary contact normal
      math::Vec3f const contact_normal{1.0f, 0.0f, 0.0f};
      auto const penetration_depth = contact_distance;
      auto const normalizing_factor =
          1.0f / (particle_a->inverse_mass + particle_b->inverse_mass);
      particle_a->current_position -= normalizing_factor *
                                      particle_a->inverse_mass *
                                      penetration_depth * contact_normal;
      particle_b->current_position += normalizing_factor *
                                      particle_b->inverse_mass *
                                      penetration_depth * contact_normal;
    } else {
      auto const distance = std::sqrt(distance2);
      auto const contact_normal = displacement / distance;
      auto const penetration_depth = contact_distance - distance;
      auto const normalizing_factor =
          1.0f / (particle_a->inverse_mass + particle_b->inverse_mass);
      particle_a->current_position -= normalizing_factor *
                                      particle_a->inverse_mass *
                                      penetration_depth * contact_normal;
      particle_b->current_position += normalizing_factor *
                                      particle_b->inverse_mass *
                                      penetration_depth * contact_normal;
    }
  }
}
} // namespace physics
} // namespace marlon