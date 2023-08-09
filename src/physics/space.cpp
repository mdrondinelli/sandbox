#include "space.h"

#include <ankerl/unordered_dense.h>

#include "bounds_tree.h"

namespace marlon {
namespace physics {
namespace {
using Bounds_tree_leaf_payload =
    std::variant<Particle_reference, Static_rigid_body_reference>;

struct Particle {
  Bounds_tree<Bounds_tree_leaf_payload>::Node *bounds_tree_leaf;
  Particle_motion_callback *motion_callback;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f current_position;
  math::Vec3f velocity;
  float mass;
  float mass_inverse;
  float radius;
  float static_friction_coefficient;
  float dynamic_friction_coefficient;
  float restitution_coefficient;
};

struct Static_rigid_body {
  Bounds_tree<Bounds_tree_leaf_payload>::Node *bounds_tree_leaf;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Mat3x4f transform;
  math::Mat3x4f transform_inverse;
  Shape shape;
  float static_friction_coefficient;
  float dynamic_friction_coefficient;
  float restitution_coefficient;
};

struct Particle_particle_contact {
  std::array<Particle *, 2> particles;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};

struct Particle_static_rigid_body_contact {
  Particle *particle;
  Static_rigid_body *static_rigid_body;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};
} // namespace

class Space::Impl {
public:
  explicit Impl(Space_create_info const &create_info)
      : _gravitational_acceleration{create_info.gravitational_acceleration} {}

  Particle_reference create_particle(Particle_create_info const &create_info) {
    auto const bounds =
        Bounds{create_info.position - math::Vec3f{create_info.radius,
                                                  create_info.radius,
                                                  create_info.radius},
               create_info.position + math::Vec3f{create_info.radius,
                                                  create_info.radius,
                                                  create_info.radius}};
    Particle_reference const reference{_next_particle_reference_value};
    Particle const value{
        .bounds_tree_leaf = _bounds_tree.create_leaf(bounds, reference),
        .motion_callback = create_info.motion_callback,
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .previous_position = create_info.position,
        .current_position = create_info.position,
        .velocity = create_info.velocity,
        .mass = create_info.mass,
        .mass_inverse = 1.0f / create_info.mass,
        .radius = create_info.radius,
        .static_friction_coefficient = create_info.static_friction_coefficient,
        .dynamic_friction_coefficient =
            create_info.dynamic_friction_coefficient,
        .restitution_coefficient = create_info.restitution_coefficient};
    try {
      _particles.emplace(reference, value);
    } catch (...) {
      _bounds_tree.destroy_leaf(value.bounds_tree_leaf);
      throw;
    }
    ++_next_particle_reference_value;
    return reference;
  }

  void destroy_particle(Particle_reference reference) {
    auto const it = _particles.find(reference);
    _bounds_tree.destroy_leaf(it->second.bounds_tree_leaf);
    _particles.erase(it);
  }

  Static_rigid_body_reference
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    auto const transform = math::make_rigid_transform_mat3x4(
        create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    Static_rigid_body_reference const reference{
        _next_static_rigid_body_reference_value};
    Static_rigid_body const value{
        .bounds_tree_leaf = _bounds_tree.create_leaf(
            physics::bounds(create_info.shape, transform), reference),
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .transform = transform,
        .transform_inverse = transform_inverse,
        .shape = create_info.shape,
        .static_friction_coefficient = create_info.static_friction_coefficient,
        .dynamic_friction_coefficient =
            create_info.dynamic_friction_coefficient,
        .restitution_coefficient = create_info.restitution_coefficient};
    try {
      _static_rigid_bodies.emplace(reference, value);
    } catch (...) {
      _bounds_tree.destroy_leaf(value.bounds_tree_leaf);
      throw;
    }
    ++_next_static_rigid_body_reference_value;
    return reference;
  }

  void destroy_static_rigid_body(Static_rigid_body_reference reference) {
    auto const it = _static_rigid_bodies.find(reference);
    _bounds_tree.destroy_leaf(it->second.bounds_tree_leaf);
    _static_rigid_bodies.erase(it);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const h_inv = 1.0f / h;
    find_contacts(simulate_info.delta_time);
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      for (auto &element : _particles) {
        auto &particle = element.second;
        particle.previous_position = particle.current_position;
        particle.velocity += h * _gravitational_acceleration;
        particle.current_position += h * particle.velocity;
      }
      solve_particle_particle_contact_positions();
      solve_particle_static_rigid_body_contact_positions();
      for (auto &element : _particles) {
        auto &particle = element.second;
        particle.velocity =
            h_inv * (particle.current_position - particle.previous_position);
      }
      solve_particle_particle_contact_velocities(h);
      solve_particle_static_rigid_body_contact_velocities(h);
    }
    for (auto &[reference, value] : _particles) {
      if (value.motion_callback != nullptr) {
        value.motion_callback->on_particle_motion(
            {reference, value.current_position, value.velocity});
      }
    }
  }

private:
  void find_contacts(float delta_time) {
    auto const safety_factor = 2.0f;
    auto const gravity_term =
        math::length(_gravitational_acceleration) * delta_time * delta_time;
    for (auto const &element : _particles) {
      auto const &particle = element.second;
      auto const radius =
          particle.radius +
          safety_factor * delta_time * math::length(particle.velocity) +
          gravity_term;
      auto const half_extents = math::Vec3f{radius, radius, radius};
      particle.bounds_tree_leaf->bounds = {
          particle.current_position - half_extents,
          particle.current_position + half_extents};
    }
    _particle_particle_contacts.clear();
    _particle_static_rigid_body_contacts.clear();
    _bounds_tree.build();
    _bounds_tree.for_each_overlapping_leaf_pair(
        [this](Bounds_tree_leaf_payload const &first_reference,
               Bounds_tree_leaf_payload const &second_reference) {
          if (first_reference.index() == 0) {
            if (second_reference.index() == 0) {
              _particle_particle_contacts.emplace_back(
                  Particle_particle_contact{
                      .particles =
                          {&_particles.at(std::get<0>(first_reference)),
                           &_particles.at(std::get<0>(second_reference))},
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f});
            } else {
              _particle_static_rigid_body_contacts.emplace_back(
                  Particle_static_rigid_body_contact{
                      .particle = &_particles.at(std::get<0>(first_reference)),
                      .static_rigid_body = &_static_rigid_bodies.at(
                          std::get<1>(second_reference)),
                      .normal = math::Vec3f::zero(),
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f,
                      .separating_velocity = 0.0f});
            }
          } else {
            if (second_reference.index() == 0) {
              _particle_static_rigid_body_contacts.emplace_back(
                  Particle_static_rigid_body_contact{
                      .particle = &_particles.at(std::get<0>(second_reference)),
                      .static_rigid_body = &_static_rigid_bodies.at(
                          std::get<1>(first_reference)),
                      .normal = math::Vec3f::zero(),
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f,
                      .separating_velocity = 0.0f});
            } else {
              // ignore static rigid bodies colliding with each other
            }
          }
        });
  }

  void solve_particle_particle_contact_positions() {
    for (auto &contact : _particle_particle_contacts) {
      auto const particle_a = contact.particles[0];
      auto const particle_b = contact.particles[1];
      auto const displacement =
          particle_a->current_position - particle_b->current_position;
      auto const distance2 = math::length2(displacement);
      auto const contact_distance = particle_a->radius + particle_b->radius;
      auto const contact_distance2 = contact_distance * contact_distance;
      if (distance2 < contact_distance2) {
        auto const [normal, separation] = [&]() {
          if (distance2 == 0.0f) {
            // particles coincide, pick arbitrary contact normal
            math::Vec3f const contact_normal{1.0f, 0.0f, 0.0f};
            auto const separation = -contact_distance;
            return std::tuple{contact_normal, separation};
          } else {
            auto const distance = std::sqrt(distance2);
            auto const normal = displacement / distance;
            auto const separation = distance - contact_distance;
            return std::tuple{normal, separation};
          }
        }();
        auto const delta_normal_force_lagrange =
            -separation / (particle_a->mass_inverse + particle_b->mass_inverse);
        auto const relative_motion =
            (particle_a->current_position - particle_a->previous_position) -
            (particle_b->current_position - particle_b->previous_position);
        auto const relative_tangential_motion =
            relative_motion - math::dot(relative_motion, normal) * normal;
        auto const delta_tangent_force_lagrange =
            math::length(relative_tangential_motion) /
            (particle_a->mass_inverse + particle_b->mass_inverse);
        contact.normal = normal;
        contact.normal_force_lagrange = delta_normal_force_lagrange;
        contact.tangent_force_lagrange = delta_tangent_force_lagrange;
        contact.separating_velocity =
            math::dot(particle_a->velocity - particle_b->velocity, normal);
        auto const normal_positional_impulse =
            delta_normal_force_lagrange * normal;
        particle_a->current_position +=
            normal_positional_impulse * particle_a->mass_inverse;
        particle_b->current_position -=
            normal_positional_impulse * particle_b->mass_inverse;
        auto const static_friction_coefficient =
            0.5f * (particle_a->static_friction_coefficient +
                    particle_b->static_friction_coefficient);
        if (contact.tangent_force_lagrange <
            static_friction_coefficient * contact.normal_force_lagrange) {
          auto const tangent_positional_impulse =
              -relative_tangential_motion /
              (particle_a->mass_inverse + particle_b->mass_inverse);
          particle_a->current_position +=
              tangent_positional_impulse * particle_a->mass_inverse;
          particle_b->current_position -=
              tangent_positional_impulse * particle_b->mass_inverse;
        }
      } else {
        contact.normal_force_lagrange = 0.0f;
        contact.tangent_force_lagrange = 0.0f;
      }
    }
  }

  void solve_particle_static_rigid_body_contact_positions() {
    for (auto &contact : _particle_static_rigid_body_contacts) {
      auto const particle = contact.particle;
      auto const static_rigid_body = contact.static_rigid_body;
      if (auto const contact_geometry = find_particle_contact(
              particle->current_position, particle->radius,
              static_rigid_body->shape, static_rigid_body->transform,
              static_rigid_body->transform_inverse)) {
        auto const delta_normal_force_lagrange =
            -contact_geometry->separation * particle->mass;
        auto const relative_motion =
            particle->current_position - particle->previous_position;
        auto const relative_tangential_motion =
            relative_motion -
            math::dot(relative_motion, contact_geometry->normal) *
                contact_geometry->normal;
        auto const delta_tangent_force_lagrange =
            math::length(relative_tangential_motion) * particle->mass;
        contact.normal = contact_geometry->normal;
        contact.normal_force_lagrange = delta_normal_force_lagrange;
        contact.tangent_force_lagrange = delta_tangent_force_lagrange;
        contact.separating_velocity =
            math::dot(contact.particle->velocity, contact.normal);
        particle->current_position +=
            -contact_geometry->separation * contact_geometry->normal;
        auto const static_friction_coefficient =
            0.5f * (particle->static_friction_coefficient +
                    static_rigid_body->static_friction_coefficient);
        if (contact.tangent_force_lagrange <
            static_friction_coefficient * contact.normal_force_lagrange) {
          particle->current_position -= relative_tangential_motion;
        }
      } else {
        contact.normal_force_lagrange = 0.0f;
        contact.tangent_force_lagrange = 0.0f;
      }
    }
  }

  void solve_particle_particle_contact_velocities(float h) {
    auto const restitution_threshold =
        2.0f * math::length(_gravitational_acceleration) * h;
    for (auto &contact : _particle_particle_contacts) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v =
            contact.particles[0]->velocity - contact.particles[1]->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f * (contact.particles[0]->dynamic_friction_coefficient +
                      contact.particles[1]->dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(dynamic_friction_coefficient *
                           contact.normal_force_lagrange / h,
                       v_t_length);
          auto const friction_velocity_impulse =
              friction_delta_velocity / (contact.particles[0]->mass_inverse +
                                         contact.particles[1]->mass_inverse);
          contact.particles[0]->velocity +=
              friction_velocity_impulse * contact.particles[0]->mass_inverse;
          contact.particles[1]->velocity -=
              friction_velocity_impulse * contact.particles[1]->mass_inverse;
        }
        auto const restitution_coefficient =
            std::abs(v_n) > restitution_threshold
                ? 0.5f * (contact.particles[0]->restitution_coefficient +
                          contact.particles[1]->restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.normal * (-v_n + std::max(-restitution_coefficient *
                                                  contact.separating_velocity,
                                              0.0f));
        auto const restitution_velocity_impulse =
            restitution_delta_velocity / (contact.particles[0]->mass_inverse +
                                          contact.particles[1]->mass_inverse);
        contact.particles[0]->velocity +=
            restitution_velocity_impulse * contact.particles[0]->mass_inverse;
        contact.particles[1]->velocity -=
            restitution_velocity_impulse * contact.particles[1]->mass_inverse;
      }
    }
  }

  void solve_particle_static_rigid_body_contact_velocities(float h) {
    auto const restitution_threshold =
        2.0f * math::length(_gravitational_acceleration) * h;
    for (auto &contact : _particle_static_rigid_body_contacts) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v = contact.particle->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f * (contact.particle->dynamic_friction_coefficient +
                      contact.static_rigid_body->dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(dynamic_friction_coefficient *
                           contact.normal_force_lagrange / h,
                       v_t_length);
          contact.particle->velocity += friction_delta_velocity;
        }
        auto const restitution_coefficient =
            std::abs(v_n) > restitution_threshold
                ? 0.5f * (contact.particle->restitution_coefficient +
                          contact.static_rigid_body->restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.normal * (-v_n + std::max(-restitution_coefficient *
                                                  contact.separating_velocity,
                                              0.0f));
        contact.particle->velocity += restitution_delta_velocity;
      }
    }
  }

  Bounds_tree<Bounds_tree_leaf_payload> _bounds_tree;
  ankerl::unordered_dense::map<Particle_reference, Particle> _particles;
  ankerl::unordered_dense::map<Static_rigid_body_reference, Static_rigid_body>
      _static_rigid_bodies;
  std::vector<Particle_particle_contact> _particle_particle_contacts;
  std::vector<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_contacts;
  std::uint64_t _next_particle_reference_value{};
  std::uint64_t _next_static_rigid_body_reference_value{};
  math::Vec3f _gravitational_acceleration;
};

Space::Space(Space_create_info const &create_info)
    : _impl{std::make_unique<Impl>(create_info)} {}

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

void Space::simulate(Space_simulate_info const &simulate_info) {
  _impl->simulate(simulate_info);
}
} // namespace physics
} // namespace marlon