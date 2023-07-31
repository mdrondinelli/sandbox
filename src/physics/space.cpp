#include "space.h"

#include <unordered_map>

namespace marlon {
namespace physics {
namespace {
constexpr auto particle_contact_offset = 0.1f;
}
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
  Static_rigid_body_reference const reference{
      _next_static_rigid_body_reference_value};
  auto const transform = math::make_rigid_transform_mat3x4(
      create_info.position, create_info.orientation);
  auto const transform_inverse = math::rigid_inverse(transform);
  Static_rigid_body const value{.collision_flags = create_info.collision_flags,
                                .collision_mask = create_info.collision_mask,
                                .transform = transform,
                                .transform_inverse = transform_inverse,
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
  flatten_particles();
  flatten_static_rigid_bodies();
  find_particle_particle_collisions();
  find_particle_static_rigid_body_collisions();
  for (auto i = 0; i < simulate_info.substep_count; ++i) {
    for (auto &[reference, value] : _particles) {
      value.previous_position = value.current_position;
      value.velocity += h * value.acceleration;
      value.current_position += h * value.velocity;
    }
    solve_particle_particle_collisions();
    solve_particle_static_rigid_body_collisions();
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

void Space::flatten_particles() {
  _flattened_particles.clear();
  _flattened_particles.reserve(_particles.size());
  for (auto &pair : _particles) {
    _flattened_particles.emplace_back(&pair.second);
  }
}

void Space::flatten_static_rigid_bodies() {
  _flattened_static_rigid_bodies.clear();
  _flattened_static_rigid_bodies.reserve(_static_rigid_bodies.size());
  for (auto &pair : _static_rigid_bodies) {
    _flattened_static_rigid_bodies.emplace_back(&pair.second);
  }
}

void Space::find_particle_particle_collisions() {
  _particle_particle_collisions.clear();
  for (std::size_t i{}; i + 1 < _flattened_particles.size(); ++i) {
    for (std::size_t j{i + 1}; j < _flattened_particles.size(); ++j) {
      auto const particle_a = _flattened_particles[i];
      auto const particle_b = _flattened_particles[j];
      if ((particle_a->collision_mask & particle_b->collision_flags) &&
          (particle_b->collision_mask & particle_a->collision_flags)) {
        auto const displacement =
            particle_b->current_position - particle_a->current_position;
        auto const distance2 = math::length2(displacement);
        auto const contact_distance = particle_a->radius + particle_b->radius +
                                      2.0f * particle_contact_offset;
        auto const contact_distance2 = contact_distance * contact_distance;
        if (distance2 < contact_distance2) {
          _particle_particle_collisions.emplace_back(particle_a, particle_b);
        }
      }
    }
  }
}

void Space::find_particle_static_rigid_body_collisions() {
  _particle_static_rigid_body_collisions.clear();
  for (std::size_t i{}; i < _flattened_particles.size(); ++i) {
    for (std::size_t j{}; j < _flattened_static_rigid_bodies.size(); ++j) {
      auto const particle = _flattened_particles[i];
      auto const static_rigid_body = _flattened_static_rigid_bodies[j];
      if ((particle->collision_mask & static_rigid_body->collision_flags) &&
          (static_rigid_body->collision_mask & particle->collision_flags)) {
        if (auto const contact = static_rigid_body->shape->collide_particle(
                static_rigid_body->transform,
                static_rigid_body->transform_inverse,
                particle->current_position,
                particle->radius + particle_contact_offset)) {
          _particle_static_rigid_body_collisions.emplace_back(
              particle, static_rigid_body);
        }
      }
    }
  }
}

void Space::solve_particle_particle_collisions() {
  for (auto const [particle_a, particle_b] : _particle_particle_collisions) {
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

void Space::solve_particle_static_rigid_body_collisions() {
  for (auto const [particle, static_rigid_body] :
       _particle_static_rigid_body_collisions) {
    if (auto contact = static_rigid_body->shape->collide_particle(
            static_rigid_body->transform, static_rigid_body->transform_inverse,
            particle->current_position, particle->radius)) {
      particle->current_position += contact->depth * contact->normal;
    }
  }
}
} // namespace physics
} // namespace marlon