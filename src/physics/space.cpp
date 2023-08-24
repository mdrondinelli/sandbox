#include "space.h"

#include <iostream>

#include <ankerl/unordered_dense.h>

#include "aabb_tree.h"

namespace marlon {
namespace physics {
namespace {
struct Particle_data;

struct Static_rigid_body_data;

struct Dynamic_rigid_body_data;

using Aabb_tree_payload_t =
    std::variant<Particle_data *, Static_rigid_body_data *,
                 Dynamic_rigid_body_data *>;

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
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
      : _data{std::make_unique<std::byte[]>(size * sizeof(Particle_data))},
        _free_indices(size), _occupancy_bits(size) {
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
    return reinterpret_cast<Particle_data *>(_data.get()) + handle.value;
  }

  template <typename F> void for_each(F &&f) {
    auto const n = static_cast<std::ptrdiff_t>(_occupancy_bits.size());
    auto const m = static_cast<std::ptrdiff_t>(_occupancy_bits.size() -
                                               _free_indices.size());
    auto k = std::ptrdiff_t{};
    for (auto i = std::ptrdiff_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        f(Particle_handle{i}, data({i}));
        if (++k == m) {
          return;
        }
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::ptrdiff_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

struct Static_rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Mat3x4f transform;
  math::Mat3x4f transform_inverse;
  Shape shape;
  Material material;
};

class Static_rigid_body_storage {
public:
  explicit Static_rigid_body_storage(std::ptrdiff_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Static_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::ptrdiff_t{}; i != size; ++i) {
      _free_indices[i] = size - i - 1;
    }
  }

  Static_rigid_body_handle alloc() {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for static rigid bodies"};
    }
    auto const index = _free_indices.back();
    auto const handle = Static_rigid_body_handle{index};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return handle;
  }

  void free(Static_rigid_body_handle handle) {
    _free_indices.emplace_back(handle.value);
    _occupancy_bits[handle.value] = false;
  }

  Static_rigid_body_data *data(Static_rigid_body_handle handle) {
    return reinterpret_cast<Static_rigid_body_data *>(_data.get()) +
           handle.value;
  }

  template <typename F> void for_each(F &&f) {
    auto const n = static_cast<std::ptrdiff_t>(_occupancy_bits.size());
    auto const m = static_cast<std::ptrdiff_t>(_occupancy_bits.size() -
                                               _free_indices.size());
    auto k = std::ptrdiff_t{};
    for (auto i = std::ptrdiff_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        f(Static_rigid_body_handle{i}, data({i}));
        if (++k == m) {
          return;
        }
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::ptrdiff_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

struct Dynamic_rigid_body_data {
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
  float inverse_mass;
  math::Mat3x3f inertia_tensor;
  math::Mat3x3f inverse_inertia_tensor;
  Shape shape;
  Material material;
  Aabb_tree<Aabb_tree_payload_t>::Node *broadphase_node;
};

class Dynamic_rigid_body_storage {
public:
  explicit Dynamic_rigid_body_storage(std::ptrdiff_t size)
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Dynamic_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::ptrdiff_t{}; i != size; ++i) {
      _free_indices[i] = size - i - 1;
    }
  }

  Dynamic_rigid_body_handle alloc() {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for static rigid bodies"};
    }
    auto const index = _free_indices.back();
    auto const handle = Dynamic_rigid_body_handle{index};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return handle;
  }

  void free(Dynamic_rigid_body_handle handle) {
    _free_indices.emplace_back(handle.value);
    _occupancy_bits[handle.value] = false;
  }

  Dynamic_rigid_body_data *data(Dynamic_rigid_body_handle handle) {
    return reinterpret_cast<Dynamic_rigid_body_data *>(_data.get()) +
           handle.value;
  }

  template <typename F> void for_each(F &&f) {
    auto const n = static_cast<std::ptrdiff_t>(_occupancy_bits.size());
    auto const m = static_cast<std::ptrdiff_t>(_occupancy_bits.size() -
                                               _free_indices.size());
    auto k = std::ptrdiff_t{};
    for (auto i = std::ptrdiff_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        f(Dynamic_rigid_body_handle{i}, data({i}));
        if (++k == m) {
          return;
        }
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::ptrdiff_t> _free_indices;
  std::vector<bool> _occupancy_bits;
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
  Static_rigid_body_data *static_rigid_body;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};

struct Particle_dynamic_rigid_body_contact {
  Particle_data *particle;
  Dynamic_rigid_body_data *body;
  math::Vec3f r_2;
  math::Vec3f n;
  float w_sum_inverse;
  float lambda_n;
  float lambda_t;
  float v_n;
};
} // namespace

class Space::Impl {
public:
  explicit Impl(Space_create_info const &create_info)
      : _particles{create_info.max_particles},
        _static_rigid_bodies{create_info.max_static_rigid_bodies},
        _dynamic_rigid_bodies{create_info.max_dynamic_rigid_bodies},
        _gravitational_acceleration{create_info.gravitational_acceleration} {}

  Particle_handle create_particle(Particle_create_info const &create_info) {
    auto const bounds =
        Aabb{create_info.position - math::Vec3f::all(create_info.radius),
             create_info.position + math::Vec3f::all(create_info.radius)};
    auto const handle = _particles.alloc();
    auto const data = _particles.data(handle);
    // TODO: cleanup allocated handle if create_leaf fails
    new (data)
        Particle_data{.aabb_tree_node = _aabb_tree.create_leaf(bounds, data),
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
    _aabb_tree.destroy_leaf(_particles.data(handle)->aabb_tree_node);
    _particles.free(handle);
  }

  Static_rigid_body_handle
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    // TODO: cleanup allocated handle if create_leaf fails
    auto const handle = _static_rigid_bodies.alloc();
    auto const data = _static_rigid_bodies.data(handle);
    new (data) Static_rigid_body_data{
        .aabb_tree_node = _aabb_tree.create_leaf(
            physics::bounds(create_info.shape, transform), data),
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .transform = transform,
        .transform_inverse = transform_inverse,
        .shape = create_info.shape,
        .material = create_info.material};
    return handle;
  }

  void destroy_static_rigid_body(Static_rigid_body_handle handle) {
    _aabb_tree.destroy_leaf(_static_rigid_bodies.data(handle)->aabb_tree_node);
    _static_rigid_bodies.free(handle);
  }

  Dynamic_rigid_body_handle
  create_dynamic_rigid_body(Dynamic_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const handle = _dynamic_rigid_bodies.alloc();
    auto const data = _dynamic_rigid_bodies.data(handle);
    auto const bounds = physics::bounds(create_info.shape, transform);
    new (data) Dynamic_rigid_body_data{
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
        .inverse_mass = 1.0f / create_info.mass,
        .inertia_tensor = create_info.inertia_tensor,
        .inverse_inertia_tensor = inverse(create_info.inertia_tensor),
        .shape = create_info.shape,
        .material = create_info.material,
        .broadphase_node = _aabb_tree.create_leaf(bounds, data)};
    return handle;
  }

  void destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle) {
    _aabb_tree.destroy_leaf(
        _dynamic_rigid_bodies.data(handle)->broadphase_node);
    _dynamic_rigid_bodies.free(handle);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const h_inv = 1.0f / h;
    build_aabb_tree(simulate_info.delta_time);
    find_potential_contacts();
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate_particles(h);
      integrate_dynamic_rigid_bodies(h);
      solve_particle_particle_contact_positions();
      solve_particle_static_rigid_body_contact_positions();
      solve_particle_dynamic_rigid_body_contact_positions();
      _particles.for_each([&](Particle_handle, Particle_data *data) {
        data->velocity =
            h_inv * (data->current_position - data->previous_position);
      });
      _dynamic_rigid_bodies.for_each(
          [&](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
            data->velocity =
                h_inv * (data->current_position - data->previous_position);
            auto const delta_orientation =
                data->current_orientation *
                math::inverse(data->previous_orientation);
            data->angular_velocity = 2.0f * h_inv * delta_orientation.v;
            data->angular_velocity = delta_orientation.w >= 0.0f
                                         ? data->angular_velocity
                                         : -data->angular_velocity;
          });
      solve_particle_particle_contact_velocities(h);
      solve_particle_static_rigid_body_contact_velocities(h);
      solve_particle_dymamic_rigid_body_contact_velocities(h);
    }
    _particles.for_each([&](Particle_handle handle, Particle_data *data) {
      if (data->motion_callback != nullptr) {
        data->motion_callback->on_particle_motion(
            {handle, data->current_position});
      }
      data->velocity =
          h_inv * (data->current_position - data->previous_position);
    });
    _dynamic_rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle handle, Dynamic_rigid_body_data *data) {
          if (data->motion_callback != nullptr) {
            data->motion_callback->on_dynamic_rigid_body_motion(
                {handle, data->current_position, data->current_orientation});
          }
        });
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const velocity_safety_factor = 2.0f;
    auto const gravity_safety_term =
        math::length(_gravitational_acceleration) * delta_time * delta_time;
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      auto const half_extents = math::Vec3f::all(
          data->radius +
          velocity_safety_factor * delta_time * math::length(data->velocity) +
          gravity_safety_term);
      data->aabb_tree_node->bounds = {data->current_position - half_extents,
                                      data->current_position + half_extents};
    });
    _dynamic_rigid_bodies.for_each([&](Dynamic_rigid_body_handle,
                                       Dynamic_rigid_body_data *data) {
      data->broadphase_node->bounds = expand(
          bounds(data->shape, math::Mat3x4f::rigid(data->current_position,
                                                   data->current_orientation)),
          velocity_safety_factor * delta_time * math::length(data->velocity) +
              gravity_safety_term);
    });
    _aabb_tree.build();
  }

  void find_potential_contacts() {
    _particle_particle_contacts.clear();
    _particle_static_rigid_body_contacts.clear();
    _particle_dynamic_rigid_body_contacts.clear();
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
                      _particle_particle_contacts.push_back(
                          {.particles = {first_handle, second_handle}});
                    } else if constexpr (std::is_same_v<
                                             U, Static_rigid_body_data *>) {
                      _particle_static_rigid_body_contacts.push_back(
                          {.particle = first_handle,
                           .static_rigid_body = second_handle});
                    } else {
                      static_assert(
                          std::is_same_v<U, Dynamic_rigid_body_data *>);
                      _particle_dynamic_rigid_body_contacts.push_back(
                          {.particle = first_handle, .body = second_handle});
                    }
                  },
                  second_payload);
            } else if constexpr (std::is_same_v<T, Static_rigid_body_data *>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_data *>) {
                      _particle_static_rigid_body_contacts.push_back(
                          {.particle = second_handle,
                           .static_rigid_body = first_handle});
                    }
                  },
                  second_payload);
            } else {
              static_assert(std::is_same_v<T, Dynamic_rigid_body_data *>);
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_data *>) {
                      _particle_dynamic_rigid_body_contacts.push_back(
                          {.particle = second_handle, .body = first_handle});
                    }
                  },
                  second_payload);
            }
          },
          first_payload);
    });
  }

  void integrate_particles(float h) {
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      data->previous_position = data->current_position;
      data->velocity += h * _gravitational_acceleration;
      data->current_position += h * data->velocity;
    });
  }

  void integrate_dynamic_rigid_bodies(float h) {
    _dynamic_rigid_bodies.for_each([&](Dynamic_rigid_body_handle,
                                       Dynamic_rigid_body_data *data) {
      data->previous_position = data->current_position;
      data->velocity += h * _gravitational_acceleration;
      data->current_position += h * data->velocity;
      data->previous_orientation = data->current_orientation;
      data->current_orientation += 0.5f * h *
                                   math::Quatf{0.0f, data->angular_velocity} *
                                   data->current_orientation;
      data->current_orientation = math::normalize(data->current_orientation);
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
        auto const [n, c] = [&]() {
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
        auto const w_sum_inverse =
            1.0f / (particle_a->mass_inverse + particle_b->mass_inverse);
        auto const delta_lambda_n = -c * w_sum_inverse;
        auto const delta_p =
            (particle_a->current_position - particle_a->previous_position) -
            (particle_b->current_position - particle_b->previous_position);
        auto const delta_p_t = delta_p - math::dot(delta_p, n) * n;
        auto const delta_lambda_t = math::length(delta_p_t) * w_sum_inverse;
        auto const mu_s =
            0.5f * (particle_a->material.static_friction_coefficient +
                    particle_b->material.static_friction_coefficient);
        auto p = delta_lambda_n * n;
        if (delta_lambda_t < mu_s * delta_lambda_n) {
          p -= delta_p_t * w_sum_inverse;
        }
        particle_a->current_position += p * particle_a->mass_inverse;
        particle_b->current_position -= p * particle_b->mass_inverse;
        contact.normal = n;
        contact.normal_force_lagrange = delta_lambda_n;
        contact.tangent_force_lagrange = delta_lambda_t;
        contact.separating_velocity =
            math::dot(particle_a->velocity - particle_b->velocity, n);
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
      if (auto const contact_geometry =
              find_positionless_particle_contact_geometry(
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

  void solve_particle_dynamic_rigid_body_contact_positions() {
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      auto const particle = contact.particle;
      auto const body = contact.body;
      auto const body_transform = math::Mat3x4f::rigid(
          body->current_position, body->current_orientation);
      auto const inverse_body_transform = math::rigid_inverse(body_transform);
      if (auto const geometry = find_positioned_particle_contact_geometry(
              particle->current_position, particle->radius, body->shape,
              body_transform, inverse_body_transform)) {
        auto const body_rotation =
            math::Mat3x3f::rotation(body->current_orientation);
        auto const inverse_body_rotation = math::transpose(body_rotation);
        auto const inverse_body_inertia_tensor = body_rotation *
                                                 body->inverse_inertia_tensor *
                                                 inverse_body_rotation;
        auto const previous_body_transform = math::Mat3x4f::rigid(
            body->previous_position, body->previous_orientation);
        auto const r_2 = geometry->position - body->current_position;
        auto const w_1 = particle->mass_inverse;
        auto const w_2 = body->inverse_mass +
                         math::transpose(math::cross(r_2, geometry->normal)) *
                             inverse_body_inertia_tensor *
                             math::cross(r_2, geometry->normal);
        auto const w_sum_inverse = 1.0f / (w_1 + w_2);
        auto const delta_lambda_n = -geometry->separation * w_sum_inverse;
        auto const delta_p =
            (particle->current_position - particle->previous_position) -
            (geometry->position -
             previous_body_transform *
                 math::Vec4f{inverse_body_transform *
                                 math::Vec4f{geometry->position, 1.0f},
                             1.0f});
        auto const delta_p_t =
            delta_p - math::dot(delta_p, geometry->normal) * geometry->normal;
        auto const delta_lambda_t = math::length(delta_p_t) * w_sum_inverse;
        auto const mu_s =
            0.5f * (particle->material.static_friction_coefficient +
                    body->material.static_friction_coefficient);
        auto p = delta_lambda_n * geometry->normal;
        if (delta_lambda_t < mu_s * delta_lambda_n) {
          p -= delta_p_t * w_sum_inverse;
        }
        particle->current_position += p * particle->mass_inverse;
        body->current_position -= p * body->inverse_mass;
        body->current_orientation -=
            0.5f *
            math::Quatf{0.0f,
                        inverse_body_inertia_tensor * math::cross(r_2, p)} *
            body->current_orientation;
        contact.r_2 = r_2;
        contact.n = geometry->normal;
        contact.w_sum_inverse = w_sum_inverse;
        contact.lambda_n = delta_lambda_n;
        contact.lambda_t = delta_lambda_t;
        contact.v_n = math::dot(
            particle->velocity -
                (body->velocity + math::cross(body->angular_velocity, r_2)),
            geometry->normal);
      } else {
        contact.lambda_n = 0.0f;
        contact.lambda_t = 0.0f;
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
    for (auto &contact : _particle_static_rigid_body_contacts) {
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

  void solve_particle_dymamic_rigid_body_contact_velocities(float h) {
    auto const restitution_threshold =
        2.0f * math::length(_gravitational_acceleration) * h;
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      auto const particle = contact.particle;
      auto const body = contact.body;
      if (contact.lambda_n != 0.0f) {
        // auto const v = contact.particle->velocity;
        // auto const v_n = math::dot(v, contact.normal);
        auto const v =
            particle->velocity -
            (body->velocity + math::cross(body->angular_velocity, contact.r_2));
        auto const v_t = v - contact.n * contact.v_n;
        auto const v_t_length2 = math::length2(v_t);
        auto p = math::Vec3f::zero();
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const mu_k =
              0.5f * (particle->material.dynamic_friction_coefficient +
                      body->material.dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(mu_k * contact.lambda_n / h, v_t_length);
          p += friction_delta_velocity;
          // contact.particle->velocity += friction_delta_velocity;
        }
        auto const body_rotation =
            math::Mat3x3f::rotation(contact.body->current_orientation);
        auto const body_rotation_inverse = math::transpose(body_rotation);
        auto const body_inertia_tensor_inverse =
            body_rotation * contact.body->inverse_inertia_tensor *
            body_rotation_inverse;
        auto const restitution_coefficient =
            std::abs(contact.v_n) > restitution_threshold
                ? 0.5f * (contact.particle->material.restitution_coefficient +
                          contact.body->material.restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.n *
            (-contact.v_n +
             std::max(-restitution_coefficient * contact.v_n, 0.0f));
        p += restitution_delta_velocity;
        p *= contact.w_sum_inverse;
        contact.particle->velocity += p * contact.particle->mass_inverse;
        contact.body->velocity -= p * contact.body->inverse_mass;
        contact.body->angular_velocity -=
            body_inertia_tensor_inverse * math::cross(contact.r_2, p);
      }
    }
  }

  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  Particle_storage _particles;
  Static_rigid_body_storage _static_rigid_bodies;
  Dynamic_rigid_body_storage _dynamic_rigid_bodies;
  std::vector<Particle_particle_contact> _particle_particle_contacts;
  std::vector<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_contacts;
  // std::vector<std::pair<Particle_data *, Dynamic_rigid_body_data *>>
  //     _particle_dynamic_rigid_body_broadphase_pairs;
  std::vector<Particle_dynamic_rigid_body_contact>
      _particle_dynamic_rigid_body_contacts;
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