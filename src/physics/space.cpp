#include "space.h"

#include <unordered_map>

namespace marlon {
namespace physics {
Particle_reference
Space::create_particle(Particle_create_info const &create_info) {
  Particle_reference const reference{_next_particle_reference_value};
  Particle const particle{.collision_flags = create_info.collision_flags,
                          .collision_mask = create_info.collision_mask,
                          .previous_position = create_info.position,
                          .current_position = create_info.position,
                          .velocity = create_info.velocity,
                          .acceleration = create_info.acceleration,
                          .damping_factor = create_info.damping_factor,
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

Static_rigid_body_reference Space::create_static_rigid_body(
    Static_rigid_body_create_info const &create_info) {
  Static_rigid_body_reference const reference{_next_particle_reference_value};
  Static_rigid_body const value{.collision_flags = create_info.collision_flags,
                                .collision_mask = create_info.collision_mask,
                                .position = create_info.position,
                                .orientation = create_info.orientation,
                                .shape = create_info.shape};
  _static_rigid_bodies.emplace(reference, value);
  ++_next_static_rigid_body_reference_value;
  return reference;
}

void Space::destroy_static_rigid_body(
    Static_rigid_body_reference static_rigid_body) {
  _static_rigid_bodies.erase(static_rigid_body);
}

void Space::simulate(Space_simulate_info const &simulate_info) {
  auto const h = simulate_info.delta_time / simulate_info.substep_count;
  auto const h_inv = 1.0f / h;
  auto const flattened_particles = flatten_particles();
  auto const flattened_static_rigid_bodies = flatten_static_rigid_bodies();
  auto const particle_particle_collisions =
      find_particle_particle_collisions(flattened_particles);
  auto const particle_static_rigid_body_collisions =
      find_particle_static_rigid_body_collisions(flattened_particles,
                                                 flattened_static_rigid_bodies);
  for (auto i = 0; i < simulate_info.substep_count; ++i) {
    for (auto &[reference, value] : _particles) {
      value.previous_position = value.current_position;
      value.velocity += h * value.acceleration;
      value.current_position += h * value.velocity;
    }
    solve_particle_particle_collisions(particle_particle_collisions);
    solve_particle_static_rigid_body_collisions(
        particle_static_rigid_body_collisions);
    for (auto &[reference, value] : _particles) {
      value.velocity =
          h_inv * (value.current_position - value.previous_position);
    }
  }
  for (auto &[reference, value] : _particles) {
    auto const damping_factor =
        std::pow(value.damping_factor, simulate_info.delta_time);
    value.velocity *= damping_factor;
    if (value.motion_callback != nullptr) {
      value.motion_callback->on_particle_motion(
          {reference, value.current_position, value.velocity});
    }
  }
}

std::vector<Space::Particle *> Space::flatten_particles() {
  std::vector<Particle *> retval;
  retval.reserve(_particles.size());
  for (auto &pair : _particles) {
    retval.emplace_back(&pair.second);
  }
  return retval;
}

std::vector<Space::Static_rigid_body *> Space::flatten_static_rigid_bodies() {
  std::vector<Static_rigid_body *> retval;
  retval.reserve(_static_rigid_bodies.size());
  for (auto &pair : _static_rigid_bodies) {
    retval.emplace_back(&pair.second);
  }
  return retval;
}

std::vector<std::pair<Space::Particle *, Space::Particle *>>
Space::find_particle_particle_collisions(std::span<Particle *const> particles) {
  std::vector<std::pair<Particle *, Particle *>> retval;
  for (std::size_t i{}; i + 1 < particles.size(); ++i) {
    for (std::size_t j{i + 1}; j < particles.size(); ++j) {
      auto const particle_a = particles[i];
      auto const particle_b = particles[j];
      if ((particle_a->collision_mask & particle_b->collision_flags) &&
          (particle_b->collision_mask & particle_a->collision_flags)) {
        auto const displacement =
            particle_b->current_position - particle_a->current_position;
        auto const distance2 = math::length2(displacement);
        auto const contact_distance =
            particle_a->radius + particle_b->radius + 0.2f;
        auto const contact_distance2 = contact_distance * contact_distance;
        if (distance2 < contact_distance2) {
          retval.emplace_back(particle_a, particle_b);
        }
      }
    }
  }
  return retval;
}

std::vector<std::pair<Space::Particle *, Space::Static_rigid_body *>>
Space::find_particle_static_rigid_body_collisions(
    std::span<Particle *const> particles,
    std::span<Static_rigid_body *const> static_rigid_bodies) {
  std::vector<std::pair<Particle *, Static_rigid_body *>> retval;
  for (std::size_t i{}; i < particles.size(); ++i) {
    for (std::size_t j{}; j < static_rigid_bodies.size(); ++j) {
      auto const particle = particles[i];
      auto const static_rigid_body = static_rigid_bodies[j];
      if ((particle->collision_mask & static_rigid_body->collision_flags) &&
          (static_rigid_body->collision_mask & particle->collision_flags)) {
        if (auto const contact = static_rigid_body->shape->collide_particle(
                static_rigid_body->position, static_rigid_body->orientation,
                particle->current_position, particle->radius + 0.1f)) {
          retval.emplace_back(particle, static_rigid_body);
        }
      }
    }
  }
  return retval;
}

void Space::solve_particle_particle_collisions(
    std::span<std::pair<Particle *, Particle *> const> collisions) {
  for (auto const [particle_a, particle_b] : collisions) {
    auto const displacement =
        particle_b->current_position - particle_a->current_position;
    auto const distance2 = math::length2(displacement);
    auto const contact_distance = particle_a->radius + particle_b->radius;
    auto const contact_distance2 = contact_distance * contact_distance;
    if (distance2 < contact_distance2) {
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
}

void Space::solve_particle_static_rigid_body_collisions(
    std::span<std::pair<Particle *, Static_rigid_body *> const> collisions) {
  for (auto const [particle, static_rigid_body] : collisions) {
    if (auto contact = static_rigid_body->shape->collide_particle(
            static_rigid_body->position, static_rigid_body->orientation,
            particle->current_position, particle->radius)) {
      particle->current_position += contact->depth * contact->normal;
    }
  }
}
} // namespace physics
} // namespace marlon