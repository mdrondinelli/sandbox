#include "space.h"

#include <ankerl/unordered_dense.h>

#include "aabb_tree.h"

namespace marlon {
namespace physics {
namespace {
struct Particle_data;

using Aabb_tree_payload_t =
    std::variant<Particle_data *, Static_rigid_body_handle,
                 Dynamic_rigid_body_handle>;

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *bounds_tree_leaf;
  Particle_motion_callback *motion_callback;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f current_position;
  math::Vec3f velocity;
  float mass;
  float mass_inverse;
  float radius;
  Material material;
};

class Particle_storage {
public:
  explicit Particle_storage(std::ptrdiff_t size) noexcept
      : _data(size), _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::ptrdiff_t{}; i != size; ++i) {
      _free_indices[i] = size - i - 1;
    }
  }

  Particle_handle alloc() {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for particles"};
    }
    auto const index = _free_indices.back();
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return Particle_handle{index};
  }

  void free(Particle_handle handle) {
    _free_indices.emplace_back(handle.value);
    _occupancy_bits[handle.value] = false;
  }

  Particle_data *data(Particle_handle handle) {
    return _data.data() + handle.value;
  }

  template <typename F> void for_each(F &&f) {
    auto const n = static_cast<std::ptrdiff_t>(_data.size());
    auto const m =
        static_cast<std::ptrdiff_t>(_data.size() - _free_indices.size());
    auto k = std::ptrdiff_t{};
    for (auto i = std::ptrdiff_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        f(Particle_handle{i}, _data.data() + i);
        if (++k == m) {
          return;
        }
      }
    }
  }

private:
  std::vector<Particle_data> _data;
  std::vector<std::ptrdiff_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

struct Static_rigid_body {
  Aabb_tree<Aabb_tree_payload_t>::Node *bounds_tree_leaf;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Mat3x4f transform;
  math::Mat3x4f transform_inverse;
  Shape shape;
  Material material;
};

struct Dynamic_rigid_body {
  Aabb_tree<Aabb_tree_payload_t>::Node *bounds_tree_leaf;
  Dynamic_rigid_body_motion_callback *motion_callback;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f current_position;
  math::Vec3f velocity;
  math::Quatf previous_orientation;
  math::Quatf current_orientation;
  math::Vec3f angular_velocity;
  float mass;
  float mass_inverse;
  math::Mat3x3f inertia_tensor;
  math::Mat3x3f inertia_tensor_inverse;
  Shape shape;
  Material material;
};

struct Particle_particle_contact {
  std::array<Particle_data *, 2> particles;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};

struct Particle_static_rigid_body_contact {
  Particle_data *particle;
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
      : _particles{create_info.max_particles},
        _gravitational_acceleration{create_info.gravitational_acceleration} {}

  Particle_handle create_particle(Particle_create_info const &create_info) {
    auto const bounds =
        Aabb{create_info.position - math::Vec3f{create_info.radius,
                                                create_info.radius,
                                                create_info.radius},
             create_info.position + math::Vec3f{create_info.radius,
                                                create_info.radius,
                                                create_info.radius}};
    auto const handle = _particles.alloc();
    auto const data = _particles.data(handle);
    // TODO: consider what happens if create_leaf fails
    *data = {.bounds_tree_leaf = _aabb_tree.create_leaf(bounds, data),
             .motion_callback = create_info.motion_callback,
             .collision_flags = create_info.collision_flags,
             .collision_mask = create_info.collision_mask,
             .previous_position = create_info.position,
             .current_position = create_info.position,
             .velocity = create_info.velocity,
             .mass = create_info.mass,
             .mass_inverse = 1.0f / create_info.mass,
             .radius = create_info.radius,
             .material = create_info.material};
    return handle;
  }

  void destroy_particle(Particle_handle handle) {
    _aabb_tree.destroy_leaf(_particles.data(handle)->bounds_tree_leaf);
    _particles.free(handle);
  }

  Static_rigid_body_handle
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    Static_rigid_body_handle const reference{
        _next_static_rigid_body_handle_value};
    Static_rigid_body const value{
        .bounds_tree_leaf = _aabb_tree.create_leaf(
            physics::bounds(create_info.shape, transform), reference),
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .transform = transform,
        .transform_inverse = transform_inverse,
        .shape = create_info.shape,
        .material = create_info.material};
    try {
      _static_rigid_bodies.emplace(reference, value);
    } catch (...) {
      _aabb_tree.destroy_leaf(value.bounds_tree_leaf);
      throw;
    }
    ++_next_static_rigid_body_handle_value;
    return reference;
  }

  void destroy_static_rigid_body(Static_rigid_body_handle reference) {
    auto const it = _static_rigid_bodies.find(reference);
    _aabb_tree.destroy_leaf(it->second.bounds_tree_leaf);
    _static_rigid_bodies.erase(it);
  }

  Dynamic_rigid_body_handle
  create_dynamic_rigid_body(Dynamic_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    Dynamic_rigid_body_handle const handle{
        _next_dynamic_rigid_body_handle_value};
    Dynamic_rigid_body const value{
        .bounds_tree_leaf = _aabb_tree.create_leaf(
            physics::bounds(create_info.shape, transform), handle),
        .motion_callback = create_info.motion_callback,
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .previous_position = create_info.position,
        .current_position = create_info.position,
        .velocity = create_info.velocity,
        .previous_orientation = create_info.orientation,
        .current_orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
        .mass = create_info.mass,
        .mass_inverse = 1.0f / create_info.mass,
        .inertia_tensor = create_info.inertia_tensor,
        .inertia_tensor_inverse = inverse(create_info.inertia_tensor),
        .shape = create_info.shape,
        .material = create_info.material};
    try {
      _dynamic_rigid_bodies.emplace(handle, value);
    } catch (...) {
      _aabb_tree.destroy_leaf(value.bounds_tree_leaf);
      throw;
    }
    ++_next_dynamic_rigid_body_handle_value;
    return handle;
  }

  void destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle) {
    auto const it = _dynamic_rigid_bodies.find(handle);
    _aabb_tree.destroy_leaf(it->second.bounds_tree_leaf);
    _dynamic_rigid_bodies.erase(it);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const h_inv = 1.0f / h;
    build_aabb_tree(simulate_info.delta_time);
    find_broadphase_pairs();
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      _particles.for_each([&](Particle_handle, Particle_data *data) {
        data->previous_position = data->current_position;
        data->velocity += h * _gravitational_acceleration;
        data->current_position += h * data->velocity;
      });
      for (auto &element : _dynamic_rigid_bodies) {
        auto &body = element.second;
        body.previous_position = body.current_position;
        body.velocity += h * _gravitational_acceleration;
        body.current_position += h * body.velocity;
        body.previous_orientation = body.current_orientation;
        body.current_orientation += 0.5f * h *
                                    math::Quatf{0.0f, body.angular_velocity} *
                                    body.current_orientation;
        body.current_orientation = math::normalize(body.current_orientation);
      }
      solve_particle_particle_contact_positions();
      solve_particle_static_rigid_body_contact_positions();
      _particles.for_each([&](Particle_handle, Particle_data *data) {
        data->velocity =
            h_inv * (data->current_position - data->previous_position);
      });
      solve_particle_particle_contact_velocities(h);
      solve_particle_static_rigid_body_contact_velocities(h);
    }
    _particles.for_each([&](Particle_handle handle, Particle_data *data) {
      if (data->motion_callback != nullptr) {
        data->motion_callback->on_particle_motion(
            {handle, data->current_position});
      }
      data->velocity =
          h_inv * (data->current_position - data->previous_position);
    });
    for (auto &[handle, value] : _dynamic_rigid_bodies) {
      if (value.motion_callback != nullptr) {
        value.motion_callback->on_dynamic_rigid_body_motion(
            {handle, value.current_position, value.current_orientation});
      }
    }
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const safety_factor = 2.0f;
    auto const gravity_term =
        math::length(_gravitational_acceleration) * delta_time * delta_time;
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      auto const radius =
          data->radius +
          safety_factor * delta_time * math::length(data->velocity) +
          gravity_term;
      auto const half_extents = math::Vec3f::all(radius);
      data->bounds_tree_leaf->bounds = {data->current_position - half_extents,
                                        data->current_position + half_extents};
    });
    _aabb_tree.build();
  }

  void find_broadphase_pairs() {
    _particle_particle_broadphase_pairs.clear();
    _particle_static_rigid_body_broadphase_pairs.clear();
    _aabb_tree.for_each_overlapping_leaf_pair([this](Aabb_tree_payload_t const
                                                         &first_payload,
                                                     Aabb_tree_payload_t const
                                                         &second_payload) {
      std::visit(
          [this, &second_payload](auto &&first_handle) {
            using T = std::decay_t<decltype(first_handle)>;
            if constexpr (std::is_same_v<T, Particle_data *>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_data *>) {
                      _particle_particle_broadphase_pairs.emplace_back(
                          Particle_particle_contact{
                              .particles = {first_handle, second_handle}});
                    } else if constexpr (std::is_same_v<
                                             U, Static_rigid_body_handle>) {
                      _particle_static_rigid_body_broadphase_pairs.emplace_back(
                          Particle_static_rigid_body_contact{
                              .particle = first_handle,
                              .static_rigid_body =
                                  &_static_rigid_bodies.at(second_handle)});
                    }
                  },
                  second_payload);
            } else if constexpr (std::is_same_v<T, Static_rigid_body_handle>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_data *>) {
                      _particle_static_rigid_body_broadphase_pairs.emplace_back(
                          Particle_static_rigid_body_contact{
                              .particle = second_handle,
                              .static_rigid_body =
                                  &_static_rigid_bodies.at(first_handle)});
                    }
                  },
                  second_payload);
            }
          },
          first_payload);
    });
  }

  void solve_particle_particle_contact_positions() {
    for (auto &contact : _particle_particle_broadphase_pairs) {
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
            0.5f * (particle_a->material.static_friction_coefficient +
                    particle_b->material.static_friction_coefficient);
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
    for (auto &contact : _particle_static_rigid_body_broadphase_pairs) {
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
            0.5f * (particle->material.static_friction_coefficient +
                    static_rigid_body->material.static_friction_coefficient);
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
    for (auto &contact : _particle_particle_broadphase_pairs) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v =
            contact.particles[0]->velocity - contact.particles[1]->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f *
              (contact.particles[0]->material.dynamic_friction_coefficient +
               contact.particles[1]->material.dynamic_friction_coefficient);
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
                ? 0.5f *
                      (contact.particles[0]->material.restitution_coefficient +
                       contact.particles[1]->material.restitution_coefficient)
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
    for (auto &contact : _particle_static_rigid_body_broadphase_pairs) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v = contact.particle->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f * (contact.particle->material.dynamic_friction_coefficient +
                      contact.static_rigid_body->material
                          .dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(dynamic_friction_coefficient *
                           contact.normal_force_lagrange / h,
                       v_t_length);
          contact.particle->velocity += friction_delta_velocity;
        }
        auto const restitution_coefficient =
            std::abs(v_n) > restitution_threshold
                ? 0.5f * (contact.particle->material.restitution_coefficient +
                          contact.static_rigid_body->material
                              .restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.normal * (-v_n + std::max(-restitution_coefficient *
                                                  contact.separating_velocity,
                                              0.0f));
        contact.particle->velocity += restitution_delta_velocity;
      }
    }
  }

  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  Particle_storage _particles;
  ankerl::unordered_dense::map<Static_rigid_body_handle, Static_rigid_body>
      _static_rigid_bodies;
  ankerl::unordered_dense::map<Dynamic_rigid_body_handle, Dynamic_rigid_body>
      _dynamic_rigid_bodies;
  std::vector<Particle_particle_contact> _particle_particle_broadphase_pairs;
  std::vector<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_broadphase_pairs;
  std::uint64_t _next_static_rigid_body_handle_value{};
  std::uint64_t _next_dynamic_rigid_body_handle_value{};
  math::Vec3f _gravitational_acceleration;
};

Space::Space(Space_create_info const &create_info)
    : _impl{std::make_unique<Impl>(create_info)} {}

Space::~Space() {}

Particle_handle
Space::create_particle(Particle_create_info const &create_info) {
  return _impl->create_particle(create_info);
}

void Space::destroy_particle(Particle_handle particle) {
  _impl->destroy_particle(particle);
}

Static_rigid_body_handle Space::create_static_rigid_body(
    Static_rigid_body_create_info const &create_info) {
  return _impl->create_static_rigid_body(create_info);
}

void Space::destroy_static_rigid_body(
    Static_rigid_body_handle static_rigid_body) {
  _impl->destroy_static_rigid_body(static_rigid_body);
}

Dynamic_rigid_body_handle Space::create_dynamic_rigid_body(
    Dynamic_rigid_body_create_info const &create_info) {
  return _impl->create_dynamic_rigid_body(create_info);
}

void Space::destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle) {
  _impl->destroy_dynamic_rigid_body(handle);
}

void Space::simulate(Space_simulate_info const &simulate_info) {
  return _impl->simulate(simulate_info);
}
} // namespace physics
} // namespace marlon