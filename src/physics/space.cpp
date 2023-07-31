#include "space.h"

#include <ankerl/unordered_dense.h>

namespace marlon {
namespace physics {
namespace {
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

constexpr auto particle_contact_offset = 0.1f;
} // namespace

class Space::Impl {
public:
  Particle_reference create_particle(Particle_create_info const &create_info) {
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

  void destroy_particle(Particle_reference particle) {
    _particles.erase(particle);
  }

  Static_rigid_body_reference
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    Static_rigid_body_reference const reference{
        _next_static_rigid_body_reference_value};
    auto const transform = math::make_rigid_transform_mat3x4(
        create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    Static_rigid_body const value{.collision_flags =
                                      create_info.collision_flags,
                                  .collision_mask = create_info.collision_mask,
                                  .transform = transform,
                                  .transform_inverse = transform_inverse,
                                  .shape = create_info.shape};
    _static_rigid_bodies.emplace(reference, value);
    ++_next_static_rigid_body_reference_value;
    return reference;
  }

  void
  destroy_static_rigid_body(Static_rigid_body_reference static_rigid_body) {
    _static_rigid_bodies.erase(static_rigid_body);
  }

  void simulate(float delta_time, int substep_count) {
    auto const h = delta_time / substep_count;
    auto const h_inv = 1.0f / h;
    flatten_particles();
    flatten_static_rigid_bodies();
    find_particle_particle_collisions();
    find_particle_static_rigid_body_collisions();
    for (auto i = 0; i < substep_count; ++i) {
      for (auto const particle : _flattened_particles) {
        particle->previous_position = particle->current_position;
        particle->velocity += h * particle->acceleration;
        particle->current_position += h * particle->velocity;
      }
      solve_particle_particle_collisions();
      solve_particle_static_rigid_body_collisions();
      for (auto const particle : _flattened_particles) {
        particle->velocity =
            h_inv * (particle->current_position - particle->previous_position);
      }
    }
    for (auto &[reference, value] : _particles) {
      auto const damping_factor = std::pow(value.damping_factor, delta_time);
      value.velocity *= damping_factor;
      if (value.motion_callback != nullptr) {
        value.motion_callback->on_particle_motion(
            {reference, value.current_position, value.velocity});
      }
    }
  }

private:
  void flatten_particles() {
    _flattened_particles.clear();
    _flattened_particles.reserve(_particles.size());
    for (auto &pair : _particles) {
      _flattened_particles.emplace_back(&pair.second);
    }
  }

  void flatten_static_rigid_bodies() {
    _flattened_static_rigid_bodies.clear();
    _flattened_static_rigid_bodies.reserve(_static_rigid_bodies.size());
    for (auto &pair : _static_rigid_bodies) {
      _flattened_static_rigid_bodies.emplace_back(&pair.second);
    }
  }

  void find_particle_particle_collisions() {
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
          auto const contact_distance = particle_a->radius +
                                        particle_b->radius +
                                        2.0f * particle_contact_offset;
          auto const contact_distance2 = contact_distance * contact_distance;
          if (distance2 < contact_distance2) {
            _particle_particle_collisions.emplace_back(particle_a, particle_b);
          }
        }
      }
    }
  }

  void find_particle_static_rigid_body_collisions() {
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

  void solve_particle_particle_collisions() {
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

  void solve_particle_static_rigid_body_collisions() {
    for (auto const [particle, static_rigid_body] :
         _particle_static_rigid_body_collisions) {
      if (auto contact = static_rigid_body->shape->collide_particle(
              static_rigid_body->transform,
              static_rigid_body->transform_inverse, particle->current_position,
              particle->radius)) {
        particle->current_position += contact->depth * contact->normal;
      }
    }
  }

  ankerl::unordered_dense::map<Particle_reference, Particle> _particles;
  ankerl::unordered_dense::map<Static_rigid_body_reference, Static_rigid_body>
      _static_rigid_bodies;
  std::vector<Particle *> _flattened_particles;
  std::vector<Static_rigid_body *> _flattened_static_rigid_bodies;
  std::vector<std::pair<Particle *, Particle *>> _particle_particle_collisions;
  std::vector<std::pair<Particle *, Static_rigid_body *>>
      _particle_static_rigid_body_collisions;
  std::uint64_t _next_particle_reference_value{};
  std::uint64_t _next_static_rigid_body_reference_value{};
};

Space::Space() : _impl{std::make_unique<Impl>()} {}

Space::~Space() {}

Particle_reference
Space::create_particle(Particle_create_info const &create_info) {
  return _impl->create_particle(create_info);
}

void Space::destroy_particle(Particle_reference particle) {
  _impl->destroy_particle(particle);
}

Static_rigid_body_reference Space::create_static_rigid_body(
    Static_rigid_body_create_info const &create_info) {
  return _impl->create_static_rigid_body(create_info);
}

void Space::destroy_static_rigid_body(
    Static_rigid_body_reference static_rigid_body) {
  _impl->destroy_static_rigid_body(static_rigid_body);
}

void Space::simulate(float delta_time, int substep_count) {
  _impl->simulate(delta_time, substep_count);
}
} // namespace physics
} // namespace marlon