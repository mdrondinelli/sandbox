#include "space.h"

#include <iostream>
#include <random>

#include <ankerl/unordered_dense.h>

#include "aabb_tree.h"

namespace marlon {
namespace physics {
namespace {
using Aabb_tree_payload_t =
    std::variant<Particle_handle, Static_rigid_body_handle,
                 Dynamic_rigid_body_handle>;

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  Particle_motion_callback *motion_callback;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Vec3f position;
  math::Vec3f velocity;
  float mass;
  float inverse_mass;
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
  math::Mat3x4f inverse_transform;
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
  math::Vec3f position;
  math::Vec3f velocity;
  math::Quatf orientation;
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
  float separation;
  float separating_velocity;
};

struct Particle_static_rigid_body_contact {
  Particle_data *particle;
  Static_rigid_body_data *body;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Particle_dynamic_rigid_body_contact {
  Particle_data *particle;
  Dynamic_rigid_body_data *body;
  math::Vec3f body_relative_position;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Static_rigid_body_dynamic_rigid_body_contact {
  Static_rigid_body_data *static_body;
  Dynamic_rigid_body_data *dynamic_body;
  math::Vec3f dynamic_body_relative_position;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Dynamic_rigid_body_dynamic_rigid_body_contact {
  std::array<Dynamic_rigid_body_data *, 2> bodies;
  std::array<math::Vec3f, 2> body_relative_positions;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

auto const max_tangential_move_coefficient = 0.1f;
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
    // TODO: cleanup allocated handle if create_leaf fails
    new (_particles.data(handle))
        Particle_data{.aabb_tree_node = _aabb_tree.create_leaf(bounds, handle),
                      .motion_callback = create_info.motion_callback,
                      .collision_flags = create_info.collision_flags,
                      .collision_mask = create_info.collision_mask,
                      .position = create_info.position,
                      .velocity = create_info.velocity,
                      .mass = create_info.mass,
                      .inverse_mass = 1.0f / create_info.mass,
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
    new (_static_rigid_bodies.data(handle)) Static_rigid_body_data{
        .aabb_tree_node = _aabb_tree.create_leaf(
            physics::bounds(create_info.shape, transform), handle),
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .transform = transform,
        .inverse_transform = transform_inverse,
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
    auto const bounds = physics::bounds(create_info.shape, transform);
    new (_dynamic_rigid_bodies.data(handle)) Dynamic_rigid_body_data{
        .motion_callback = create_info.motion_callback,
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
        .mass = create_info.mass,
        .inverse_mass = 1.0f / create_info.mass,
        .inertia_tensor = create_info.inertia_tensor,
        .inverse_inertia_tensor = inverse(create_info.inertia_tensor),
        .shape = create_info.shape,
        .material = create_info.material,
        .broadphase_node = _aabb_tree.create_leaf(bounds, handle)};
    return handle;
  }

  void destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle) {
    _aabb_tree.destroy_leaf(
        _dynamic_rigid_bodies.data(handle)->broadphase_node);
    _dynamic_rigid_bodies.free(handle);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const gravitational_velocity_delta = _gravitational_acceleration * h;
    auto const max_separating_velocity_for_bounce =
        -2.0f * math::length(gravitational_velocity_delta);
    build_aabb_tree(simulate_info.delta_time);
    find_potential_contacts();
    // auto rng = std::mt19937_64{};
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate_particles(h);
      integrate_dynamic_rigid_bodies(h);
      find_contacts();
      auto const contact_count = count_contacts();
      resolve_contact_positions(contact_count);
      resolve_contact_velocities(contact_count, gravitational_velocity_delta,
                                 max_separating_velocity_for_bounce);
    }
    call_particle_motion_callbacks();
    call_dynamic_rigid_body_motion_callbacks();
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const constant_safety_term = 0.0f;
    auto const velocity_safety_factor = 2.0f;
    auto const gravity_safety_factor = 2.0f;
    auto const gravity_safety_term = gravity_safety_factor *
                                     math::length(_gravitational_acceleration) *
                                     delta_time * delta_time;
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      auto const half_extents = math::Vec3f::all(
          data->radius + constant_safety_term +
          velocity_safety_factor * math::length(data->velocity) * delta_time +
          gravity_safety_term);
      data->aabb_tree_node->bounds = {data->position - half_extents,
                                      data->position + half_extents};
    });
    _dynamic_rigid_bodies.for_each([&](Dynamic_rigid_body_handle,
                                       Dynamic_rigid_body_data *data) {
      data->broadphase_node->bounds =
          expand(bounds(data->shape, math::Mat3x4f::rigid(data->position,
                                                          data->orientation)),
                 constant_safety_term +
                     velocity_safety_factor * math::length(data->velocity) *
                         delta_time +
                     gravity_safety_term);
    });
    _aabb_tree.build();
  }

  void find_potential_contacts() {
    _potential_particle_particle_contacts.clear();
    _potential_particle_static_rigid_body_contacts.clear();
    _potential_particle_dynamic_rigid_body_contacts.clear();
    _potential_static_rigid_body_dynamic_rigid_body_contacts.clear();
    _potential_dynamic_rigid_body_dynamic_rigid_body_contacts.clear();
    _aabb_tree.for_each_overlapping_leaf_pair([this](Aabb_tree_payload_t const
                                                         &first_payload,
                                                     Aabb_tree_payload_t const
                                                         &second_payload) {
      std::visit(
          [this, &second_payload](auto &&first_handle) {
            using T = std::decay_t<decltype(first_handle)>;
            if constexpr (std::is_same_v<T, Particle_handle>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_handle>) {
                      _potential_particle_particle_contacts.push_back(
                          {first_handle, second_handle});
                    } else if constexpr (std::is_same_v<
                                             U, Static_rigid_body_handle>) {
                      _potential_particle_static_rigid_body_contacts.push_back(
                          {first_handle, second_handle});
                    } else {
                      static_assert(
                          std::is_same_v<U, Dynamic_rigid_body_handle>);
                      _potential_particle_dynamic_rigid_body_contacts.push_back(
                          {first_handle, second_handle});
                    }
                  },
                  second_payload);
            } else if constexpr (std::is_same_v<T, Static_rigid_body_handle>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_handle>) {
                      _potential_particle_static_rigid_body_contacts.push_back(
                          {second_handle, first_handle});
                    } else if constexpr (std::is_same_v<
                                             U, Dynamic_rigid_body_handle>) {
                      _potential_static_rigid_body_dynamic_rigid_body_contacts
                          .push_back({first_handle, second_handle});
                    }
                  },
                  second_payload);
            } else {
              static_assert(std::is_same_v<T, Dynamic_rigid_body_handle>);
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_handle>) {
                      _potential_particle_dynamic_rigid_body_contacts.push_back(
                          {second_handle, first_handle});
                    } else if constexpr (std::is_same_v<
                                             U, Static_rigid_body_handle>) {
                      _potential_static_rigid_body_dynamic_rigid_body_contacts
                          .push_back({second_handle, first_handle});
                    } else if constexpr (std::is_same_v<
                                             U, Dynamic_rigid_body_handle>) {
                      _potential_dynamic_rigid_body_dynamic_rigid_body_contacts
                          .push_back({first_handle, second_handle});
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
      data->velocity += h * _gravitational_acceleration;
      data->position += h * data->velocity;
    });
  }

  void integrate_dynamic_rigid_bodies(float h) {
    _dynamic_rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->velocity += h * _gravitational_acceleration;
          data->position += h * data->velocity;
          data->orientation += 0.5f *
                               math::Quatf{0.0f, h * data->angular_velocity} *
                               data->orientation;
          data->orientation = math::normalize(data->orientation);
        });
  }

  void find_contacts() {
    find_particle_particle_contacts();
    find_particle_static_rigid_body_contacts();
    find_particle_dynamic_rigid_body_contacts();
    find_static_rigid_body_dynamic_rigid_body_contacts();
    find_dynamic_rigid_body_dynamic_rigid_body_contacts();
  }

  void find_particle_particle_contacts() {
    _particle_particle_contacts.clear();
    for (auto const &contact : _potential_particle_particle_contacts) {
      auto const particles = std::array<Particle_data *, 2>{
          _particles.data(contact.first), _particles.data(contact.second)};
      auto const displacement = particles[0]->position - particles[1]->position;
      auto const distance2 = math::length_squared(displacement);
      auto const contact_distance = particles[0]->radius + particles[1]->radius;
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
        auto const separating_velocity =
            math::dot(particles[0]->velocity - particles[1]->velocity, normal);
        _particle_particle_contacts.push_back(
            {.particles = particles,
             .normal = normal,
             .separation = separation,
             .separating_velocity = separating_velocity});
      }
    }
  }

  void find_particle_static_rigid_body_contacts() {
    _particle_static_rigid_body_contacts.clear();
    for (auto const &contact : _potential_particle_static_rigid_body_contacts) {
      auto const particle = _particles.data(contact.first);
      auto const body = _static_rigid_bodies.data(contact.second);
      if (auto const contact_geometry =
              particle_shape_positionless_contact_geometry(
                  particle->position, particle->radius, body->shape,
                  body->transform, body->inverse_transform)) {
        auto const separating_velocity =
            math::dot(particle->velocity, contact_geometry->normal);
        _particle_static_rigid_body_contacts.push_back(
            {.particle = particle,
             .body = body,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
      }
    }
  }

  void find_particle_dynamic_rigid_body_contacts() {
    _particle_dynamic_rigid_body_contacts.clear();
    for (auto const &contact :
         _potential_particle_dynamic_rigid_body_contacts) {
      auto const particle = _particles.data(contact.first);
      auto const body = _dynamic_rigid_bodies.data(contact.second);
      auto const body_transform =
          math::Mat3x4f::rigid(body->position, body->orientation);
      auto const inverse_body_transform = math::rigid_inverse(body_transform);
      if (auto const contact_geometry =
              particle_shape_positioned_contact_geometry(
                  particle->position, particle->radius, body->shape,
                  body_transform, inverse_body_transform)) {
        auto const body_relative_position =
            contact_geometry->position - body->position;
        auto const relative_velocity =
            particle->velocity -
            (body->velocity +
             math::cross(body->angular_velocity, body_relative_position));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _particle_dynamic_rigid_body_contacts.push_back(
            {.particle = particle,
             .body = body,
             .body_relative_position = body_relative_position,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
      }
    }
  }

  void find_static_rigid_body_dynamic_rigid_body_contacts() {
    _static_rigid_body_dynamic_rigid_body_contacts.clear();
    for (auto const &contact :
         _potential_static_rigid_body_dynamic_rigid_body_contacts) {
      auto const static_body = _static_rigid_bodies.data(contact.first);
      auto const dynamic_body = _dynamic_rigid_bodies.data(contact.second);
      auto const dynamic_body_transform = math::Mat3x4f::rigid(
          dynamic_body->position, dynamic_body->orientation);
      auto const dynamic_body_inverse_transform =
          math::rigid_inverse(dynamic_body_transform);
      if (auto const contact_geometry = shape_shape_contact_geometry(
              static_body->shape, static_body->transform,
              static_body->inverse_transform, dynamic_body->shape,
              dynamic_body_transform, dynamic_body_inverse_transform)) {
        auto const dynamic_body_relative_contact_position =
            contact_geometry->position - dynamic_body->position;
        auto const relative_velocity =
            -(dynamic_body->velocity +
              math::cross(dynamic_body->angular_velocity,
                          dynamic_body_relative_contact_position));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _static_rigid_body_dynamic_rigid_body_contacts.push_back(
            {.static_body = static_body,
             .dynamic_body = dynamic_body,
             .dynamic_body_relative_position =
                 dynamic_body_relative_contact_position,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
      }
    }
  }

  void find_dynamic_rigid_body_dynamic_rigid_body_contacts() {
    _dynamic_rigid_body_dynamic_rigid_body_contacts.clear();
    for (auto const &contact :
         _potential_dynamic_rigid_body_dynamic_rigid_body_contacts) {
      auto const bodies = std::array<Dynamic_rigid_body_data *, 2>{
          _dynamic_rigid_bodies.data(contact.first),
          _dynamic_rigid_bodies.data(contact.second)};
      auto const body_transforms = std::array<math::Mat3x4f, 2>{
          math::Mat3x4f::rigid(bodies[0]->position, bodies[0]->orientation),
          math::Mat3x4f::rigid(bodies[1]->position, bodies[1]->orientation)};
      auto const body_inverse_transforms =
          std::array<math::Mat3x4f, 2>{math::rigid_inverse(body_transforms[0]),
                                       math::rigid_inverse(body_transforms[1])};
      if (auto const contact_geometry = shape_shape_contact_geometry(
              bodies[0]->shape, body_transforms[0], body_inverse_transforms[0],
              bodies[1]->shape, body_transforms[1],
              body_inverse_transforms[1])) {
        auto const body_relative_contact_positions = std::array<math::Vec3f, 2>{
            contact_geometry->position - bodies[0]->position,
            contact_geometry->position - bodies[1]->position};
        auto const relative_velocity =
            (bodies[0]->velocity +
             math::cross(bodies[0]->angular_velocity,
                         body_relative_contact_positions[0])) -
            (bodies[1]->velocity +
             math::cross(bodies[1]->angular_velocity,
                         body_relative_contact_positions[1]));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _dynamic_rigid_body_dynamic_rigid_body_contacts.push_back(
            {.bodies = bodies,
             .body_relative_positions = body_relative_contact_positions,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
      }
    }
  }

  int count_contacts() const {
    return static_cast<int>(
        _particle_particle_contacts.size() +
        _particle_static_rigid_body_contacts.size() +
        _particle_dynamic_rigid_body_contacts.size() +
        _static_rigid_body_dynamic_rigid_body_contacts.size() +
        _dynamic_rigid_body_dynamic_rigid_body_contacts.size());
  }

  void resolve_contact_positions(int iterations) {
    for (auto i = 0; i != iterations; ++i) {
      if (auto const contact = find_contact_of_least_negative_separation()) {
        std::visit(
            [this](auto &&arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (std::is_same_v<T, Particle_particle_contact *>) {
                resolve_particle_particle_contact_position(arg);
              } else if constexpr (std::is_same_v<
                                       T,
                                       Particle_static_rigid_body_contact *>) {
                resolve_particle_static_rigid_body_contact_position(arg);
              } else if constexpr (std::is_same_v<
                                       T,
                                       Particle_dynamic_rigid_body_contact *>) {
                resolve_particle_dynamic_rigid_body_contact_position(arg);
              } else if constexpr (
                  std::is_same_v<
                      T, Static_rigid_body_dynamic_rigid_body_contact *>) {
                resolve_static_rigid_body_dynamic_rigid_body_contact_position(
                    arg);
              } else {
                static_assert(
                    std::is_same_v<
                        T, Dynamic_rigid_body_dynamic_rigid_body_contact *>);
                resolve_dynamic_rigid_body_dynamic_rigid_body_contact_position(
                    arg);
              }
            },
            *contact);
      } else {
        break;
      }
    }
    // for (auto const &contact : _particle_particle_contacts) {
    //   resolve_particle_particle_contact_position(contact);
    // }
    // for (auto const &contact : _particle_static_rigid_body_contacts) {
    //   resolve_particle_static_rigid_body_contact_position(contact);
    // }
    // for (auto const &contact : _particle_dynamic_rigid_body_contacts) {
    //   resolve_particle_dynamic_rigid_body_contact_position(contact);
    // }
    // for (auto const &contact :
    // _static_rigid_body_dynamic_rigid_body_contacts) {
    //   resolve_static_rigid_body_dynamic_rigid_body_contact_position(contact);
    // }
    // for (auto const &contact :
    //      _dynamic_rigid_body_dynamic_rigid_body_contacts) {
    //   resolve_dynamic_rigid_body_dynamic_rigid_body_contact_position(contact);
    // }
  }

  std::optional<std::variant<Particle_particle_contact *,
                             Particle_static_rigid_body_contact *,
                             Particle_dynamic_rigid_body_contact *,
                             Static_rigid_body_dynamic_rigid_body_contact *,
                             Dynamic_rigid_body_dynamic_rigid_body_contact *>>
  find_contact_of_least_negative_separation() {
    auto retval = std::optional<std::variant<
        Particle_particle_contact *, Particle_static_rigid_body_contact *,
        Particle_dynamic_rigid_body_contact *,
        Static_rigid_body_dynamic_rigid_body_contact *,
        Dynamic_rigid_body_dynamic_rigid_body_contact *>>{};
    auto retval_separation = 0.0f;
    for (auto &contact : _particle_particle_contacts) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    }
    for (auto &contact : _particle_static_rigid_body_contacts) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    }
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    }
    for (auto &contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    }
    for (auto &contact : _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    }
    return retval;
  }

  void resolve_particle_particle_contact_position(
      Particle_particle_contact *contact) {
    auto const particles = contact->particles;
    auto const distance_per_impulse =
        particles[0]->inverse_mass + particles[1]->inverse_mass;
    auto const impulse_per_distance = 1.0f / distance_per_impulse;
    auto const impulse =
        -contact->separation * impulse_per_distance * contact->normal;
    auto const particle_position_deltas =
        std::array<math::Vec3f, 2>{impulse * particles[0]->inverse_mass,
                                   impulse * -particles[1]->inverse_mass};
    particles[0]->position += particle_position_deltas[0];
    particles[1]->position += particle_position_deltas[1];
    contact->separation = 0.0f;
    for (auto &other_contact : _particle_particle_contacts) {
      if (&other_contact != contact) {
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.particles[0] == particles[i]) {
            other_contact.separation +=
                math::dot(particle_position_deltas[i], other_contact.normal);
            break;
          }
        }
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.particles[1] == particles[i]) {
            other_contact.separation -=
                math::dot(particle_position_deltas[i], other_contact.normal);
            break;
          }
        }
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.particle == particles[i]) {
          other_contact.separation +=
              math::dot(particle_position_deltas[i], other_contact.normal);
          break;
        }
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.particle == particles[i]) {
          other_contact.separation +=
              math::dot(particle_position_deltas[i], other_contact.normal);
          break;
        }
      }
    }
  }

  void resolve_particle_static_rigid_body_contact_position(
      Particle_static_rigid_body_contact *contact) {
    auto const particle = contact->particle;
    auto const particle_position_delta = contact->normal * -contact->separation;
    particle->position += particle_position_delta;
    contact->separation = 0.0f;
    for (auto &other_contact : _particle_particle_contacts) {
      if (other_contact.particles[0] == particle) {
        other_contact.separation +=
            math::dot(particle_position_delta, other_contact.normal);
      } else if (other_contact.particles[1] == particle) {
        other_contact.separation -=
            math::dot(particle_position_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      if (&other_contact != contact) {
        if (other_contact.particle == particle) {
          other_contact.separation +=
              math::dot(particle_position_delta, other_contact.normal);
        }
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (other_contact.particle == particle) {
        other_contact.separation +=
            math::dot(particle_position_delta, other_contact.normal);
      }
    }
  }

  void resolve_particle_dynamic_rigid_body_contact_position(
      Particle_dynamic_rigid_body_contact *contact) {
    auto const particle = contact->particle;
    auto const body = contact->body;
    auto const body_rotation = math::Mat3x3f::rotation(body->orientation);
    auto const body_inverse_rotation = math::transpose(body_rotation);
    auto const body_inverse_inertia_tensor =
        body_rotation * body->inverse_inertia_tensor * body_inverse_rotation;
    auto const body_relative_contact_position = contact->body_relative_position;
    auto const body_angular_impulse_per_impulse =
        math::cross(body_relative_contact_position, contact->normal);
    auto const body_angular_velocity_per_impulse =
        body_inverse_inertia_tensor * body_angular_impulse_per_impulse;
    auto const body_tangential_velocity_per_impulse = math::cross(
        body_angular_velocity_per_impulse, body_relative_contact_position);
    auto const particle_separating_linear_velocity_per_impulse =
        particle->inverse_mass;
    auto const body_separating_linear_velocity_per_impulse = body->inverse_mass;
    auto const body_separating_tangential_velocity_per_impulse =
        math::dot(body_tangential_velocity_per_impulse, contact->normal);
    auto const separating_velocity_per_impulse =
        particle_separating_linear_velocity_per_impulse +
        body_separating_linear_velocity_per_impulse +
        body_separating_tangential_velocity_per_impulse;
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const impulse = contact->separation * impulse_per_separating_velocity;
    auto const particle_linear_move =
        impulse * -particle_separating_linear_velocity_per_impulse;
    auto const particle_position_delta = contact->normal * particle_linear_move;
    auto body_linear_move =
        impulse * body_separating_linear_velocity_per_impulse;
    auto body_tangential_move =
        impulse * body_separating_tangential_velocity_per_impulse;
    auto const body_max_tangential_move_magnitude =
        max_tangential_move_coefficient *
        math::length(body_relative_contact_position);
    if (std::abs(body_tangential_move) > body_max_tangential_move_magnitude) {
      auto const body_total_move = body_linear_move + body_tangential_move;
      body_tangential_move = std::signbit(body_tangential_move)
                                 ? -body_max_tangential_move_magnitude
                                 : body_max_tangential_move_magnitude;
      body_linear_move = body_total_move - body_tangential_move;
    }
    auto const body_position_delta = contact->normal * body_linear_move;
    auto const body_angular_velocity_per_separating_tangential_velocity =
        body_angular_velocity_per_impulse /
        body_separating_tangential_velocity_per_impulse;
    auto const body_orientation_delta =
        body_angular_velocity_per_separating_tangential_velocity *
        body_tangential_move;
    particle->position += particle_position_delta;
    body->position += body_position_delta;
    body->orientation +=
        0.5f * math::Quatf{0.0f, body_orientation_delta} * body->orientation;
    contact->separation = 0.0f;
    for (auto &other_contact : _particle_particle_contacts) {
      if (other_contact.particles[0] == particle) {
        other_contact.separation +=
            math::dot(particle_position_delta, other_contact.normal);
      } else if (other_contact.particles[1] == particle) {
        other_contact.separation -=
            math::dot(particle_position_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      if (other_contact.particle == particle) {
        other_contact.separation +=
            math::dot(particle_position_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (&other_contact != contact) {
        if (other_contact.particle == particle) {
          other_contact.separation +=
              math::dot(particle_position_delta, other_contact.normal);
        } else if (other_contact.body == body) {
          other_contact.separation -=
              math::dot(body_position_delta +
                            math::cross(body_orientation_delta,
                                        other_contact.body_relative_position),
                        other_contact.normal);
        }
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.dynamic_body == body) {
        other_contact.separation -= math::dot(
            body_position_delta +
                math::cross(body_orientation_delta,
                            other_contact.dynamic_body_relative_position),
            other_contact.normal);
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.bodies[0] == body) {
        other_contact.separation +=
            math::dot(body_position_delta +
                          math::cross(body_orientation_delta,
                                      other_contact.body_relative_positions[0]),
                      other_contact.normal);
      } else if (other_contact.bodies[1] == body) {
        other_contact.separation -=
            math::dot(body_position_delta +
                          math::cross(body_orientation_delta,
                                      other_contact.body_relative_positions[1]),
                      other_contact.normal);
      }
    }
  }

  void resolve_static_rigid_body_dynamic_rigid_body_contact_position(
      Static_rigid_body_dynamic_rigid_body_contact *contact) {
    auto const dynamic_body = contact->dynamic_body;
    auto const dynamic_body_rotation =
        math::Mat3x3f::rotation(dynamic_body->orientation);
    auto const dynamic_body_inverse_rotation =
        math::transpose(dynamic_body_rotation);
    auto const dynamic_body_inverse_inertia_tensor =
        dynamic_body_rotation * dynamic_body->inverse_inertia_tensor *
        dynamic_body_inverse_rotation;
    auto const dynamic_body_relative_contact_position =
        contact->dynamic_body_relative_position;
    auto const dynamic_body_angular_impulse_per_impulse =
        math::cross(dynamic_body_relative_contact_position, contact->normal);
    auto const dynamic_body_angular_velocity_per_impulse =
        dynamic_body_inverse_inertia_tensor *
        dynamic_body_angular_impulse_per_impulse;
    auto const dynamic_body_tangential_velocity_per_impulse =
        math::cross(dynamic_body_angular_velocity_per_impulse,
                    dynamic_body_relative_contact_position);
    auto const dynamic_body_separating_linear_velocity_per_impulse =
        dynamic_body->inverse_mass;
    auto const dynamic_body_separating_tangential_velocity_per_impulse =
        math::dot(dynamic_body_tangential_velocity_per_impulse,
                  contact->normal);
    auto const separating_velocity_per_impulse =
        dynamic_body_separating_linear_velocity_per_impulse +
        dynamic_body_separating_tangential_velocity_per_impulse;
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto dynamic_body_linear_move =
        contact->separation *
        dynamic_body_separating_linear_velocity_per_impulse *
        impulse_per_separating_velocity;
    auto dynamic_body_tangential_move =
        contact->separation *
        dynamic_body_separating_tangential_velocity_per_impulse *
        impulse_per_separating_velocity;
    auto const dynamic_body_max_angular_move_magnitude =
        max_tangential_move_coefficient *
        math::length(dynamic_body_relative_contact_position);
    if (std::abs(dynamic_body_tangential_move) >
        dynamic_body_max_angular_move_magnitude) {
      auto const dynamic_body_total_move =
          dynamic_body_linear_move + dynamic_body_tangential_move;
      dynamic_body_tangential_move =
          std::signbit(dynamic_body_tangential_move)
              ? -dynamic_body_max_angular_move_magnitude
              : dynamic_body_max_angular_move_magnitude;
      dynamic_body_linear_move =
          dynamic_body_total_move - dynamic_body_tangential_move;
    }
    auto const dynamic_body_position_delta =
        contact->normal * dynamic_body_linear_move;
    auto const
        dynamic_body_angular_velocity_per_separating_tangential_velocity =
            dynamic_body_angular_velocity_per_impulse /
            dynamic_body_separating_tangential_velocity_per_impulse;
    auto const dynamic_body_orientation_delta =
        dynamic_body_angular_velocity_per_separating_tangential_velocity *
        dynamic_body_tangential_move;
    dynamic_body->position += dynamic_body_position_delta;
    dynamic_body->orientation +=
        0.5f * math::Quatf{0.0f, dynamic_body_orientation_delta} *
        dynamic_body->orientation;
    contact->separation = 0.0f;
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (other_contact.body == dynamic_body) {
        other_contact.separation -=
            math::dot(dynamic_body_position_delta +
                          math::cross(dynamic_body_orientation_delta,
                                      other_contact.body_relative_position),
                      contact->normal);
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.dynamic_body == dynamic_body) {
        other_contact.separation -= math::dot(
            dynamic_body_position_delta +
                math::cross(dynamic_body_orientation_delta,
                            other_contact.dynamic_body_relative_position),
            contact->normal);
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.bodies[0] == dynamic_body) {
        other_contact.separation +=
            math::dot(dynamic_body_position_delta +
                          math::cross(dynamic_body_orientation_delta,
                                      other_contact.body_relative_positions[0]),
                      contact->normal);
      } else if (other_contact.bodies[1] == dynamic_body) {
        other_contact.separation -=
            math::dot(dynamic_body_position_delta +
                          math::cross(dynamic_body_orientation_delta,
                                      other_contact.body_relative_positions[1]),
                      contact->normal);
      }
    }
  }

  void resolve_dynamic_rigid_body_dynamic_rigid_body_contact_position(
      Dynamic_rigid_body_dynamic_rigid_body_contact *contact) {
    auto const bodies = contact->bodies;
    auto const body_rotations = std::array<math::Mat3x3f, 2>{
        math::Mat3x3f::rotation(bodies[0]->orientation),
        math::Mat3x3f::rotation(bodies[1]->orientation)};
    auto const body_inverse_rotations = std::array<math::Mat3x3f, 2>{
        math::transpose(body_rotations[0]), math::transpose(body_rotations[1])};
    auto const body_inverse_inertia_tensors = std::array<math::Mat3x3f, 2>{
        body_rotations[0] * bodies[0]->inverse_inertia_tensor *
            body_inverse_rotations[0],
        body_rotations[1] * bodies[1]->inverse_inertia_tensor *
            body_inverse_rotations[1]};
    auto const body_relative_contact_positions =
        contact->body_relative_positions;
    auto const body_angular_impulses_per_impulse = std::array<math::Vec3f, 2>{
        math::cross(body_relative_contact_positions[0], contact->normal),
        math::cross(body_relative_contact_positions[1], contact->normal)};
    auto const body_angular_velocities_per_impulse = std::array<math::Vec3f, 2>{
        body_inverse_inertia_tensors[0] * body_angular_impulses_per_impulse[0],
        body_inverse_inertia_tensors[1] * body_angular_impulses_per_impulse[1]};
    auto const body_tangential_velocities_per_impulse =
        std::array<math::Vec3f, 2>{
            math::cross(body_angular_velocities_per_impulse[0],
                        body_relative_contact_positions[0]),
            math::cross(body_angular_velocities_per_impulse[1],
                        body_relative_contact_positions[1])};
    auto const body_separating_linear_velocities_per_impulse =
        std::array<float, 2>{bodies[0]->inverse_mass, bodies[1]->inverse_mass};
    auto const body_separating_tangential_velocities_per_impulse =
        std::array<float, 2>{
            math::dot(body_tangential_velocities_per_impulse[0],
                      contact->normal),
            math::dot(body_tangential_velocities_per_impulse[1],
                      contact->normal)};
    auto const separating_velocity_per_impulse =
        body_separating_linear_velocities_per_impulse[0] +
        body_separating_linear_velocities_per_impulse[1] +
        body_separating_tangential_velocities_per_impulse[0] +
        body_separating_tangential_velocities_per_impulse[1];
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const impulse = contact->separation * impulse_per_separating_velocity;
    auto body_linear_moves = std::array<float, 2>{
        impulse * -body_separating_linear_velocities_per_impulse[0],
        impulse * body_separating_linear_velocities_per_impulse[1]};
    auto body_tangential_moves = std::array<float, 2>{
        impulse * -body_separating_tangential_velocities_per_impulse[0],
        impulse * body_separating_tangential_velocities_per_impulse[1]};
    auto const body_max_angular_move_magnitudes = std::array<float, 2>{
        max_tangential_move_coefficient *
            math::length(body_relative_contact_positions[0]),
        max_tangential_move_coefficient *
            math::length(body_relative_contact_positions[1])};
    for (auto i = 0; i != 2; ++i) {
      if (std::abs(body_tangential_moves[i]) >
          body_max_angular_move_magnitudes[i]) {
        auto const body_total_move =
            body_linear_moves[i] + body_tangential_moves[i];
        body_tangential_moves[i] = std::signbit(body_tangential_moves[i])
                                       ? -body_max_angular_move_magnitudes[i]
                                       : body_max_angular_move_magnitudes[i];
        body_linear_moves[i] = body_total_move - body_tangential_moves[i];
      }
    }
    auto const body_position_deltas =
        std::array<math::Vec3f, 2>{contact->normal * body_linear_moves[0],
                                   contact->normal * body_linear_moves[1]};
    auto const body_angular_velocities_per_separating_tangential_velocity =
        std::array<math::Vec3f, 2>{
            body_angular_velocities_per_impulse[0] /
                body_separating_tangential_velocities_per_impulse[0],
            body_angular_velocities_per_impulse[1] /
                body_separating_tangential_velocities_per_impulse[1]};
    auto const body_orientation_deltas = std::array<math::Vec3f, 2>{
        body_angular_velocities_per_separating_tangential_velocity[0] *
            body_tangential_moves[0],
        body_angular_velocities_per_separating_tangential_velocity[1] *
            body_tangential_moves[1]};
    for (auto i = 0; i != 2; ++i) {
      bodies[i]->position += body_position_deltas[i];
      bodies[i]->orientation += 0.5f *
                                math::Quatf{0.0f, body_orientation_deltas[i]} *
                                bodies[i]->orientation;
    }
    contact->separation = 0.0f;
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.body == bodies[i]) {
          other_contact.separation -=
              math::dot(body_position_deltas[i] +
                            math::cross(body_orientation_deltas[i],
                                        other_contact.body_relative_position),
                        contact->normal);
          break;
        }
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.dynamic_body == bodies[i]) {
          other_contact.separation -= math::dot(
              body_position_deltas[i] +
                  math::cross(body_orientation_deltas[i],
                              other_contact.dynamic_body_relative_position),
              contact->normal);
          break;
        }
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.bodies[0] == bodies[i]) {
          other_contact.separation += math::dot(
              body_position_deltas[i] +
                  math::cross(body_orientation_deltas[i],
                              other_contact.body_relative_positions[0]),
              contact->normal);
          break;
        }
      }
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.bodies[1] == bodies[i]) {
          other_contact.separation -= math::dot(
              body_position_deltas[i] +
                  math::cross(body_orientation_deltas[i],
                              other_contact.body_relative_positions[1]),
              contact->normal);
          break;
        }
      }
    }
  }

  void
  resolve_contact_velocities(int iterations,
                             math::Vec3f const &gravitational_velocity_delta,
                             float max_separating_velocity_for_bounce) {
    for (auto i = 0; i != iterations; ++i) {
      if (auto const contact =
              find_contact_of_least_negative_separating_velocity()) {
        std::visit(
            [=, this](auto &&arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (std::is_same_v<T, Particle_particle_contact *>) {
                resolve_particle_particle_contact_velocity(
                    arg, max_separating_velocity_for_bounce);
              } else if constexpr (std::is_same_v<
                                       T,
                                       Particle_static_rigid_body_contact *>) {
                resolve_particle_static_rigid_body_contact_velocity(
                    arg, gravitational_velocity_delta,
                    max_separating_velocity_for_bounce);
              } else if constexpr (std::is_same_v<
                                       T,
                                       Particle_dynamic_rigid_body_contact *>) {
                resolve_particle_dynamic_rigid_body_contact_velocity(
                    arg, max_separating_velocity_for_bounce);
              } else if constexpr (
                  std::is_same_v<
                      T, Static_rigid_body_dynamic_rigid_body_contact *>) {
                resolve_static_rigid_body_dynamic_rigid_body_contact_velocity(
                    arg, gravitational_velocity_delta,
                    max_separating_velocity_for_bounce);
              } else {
                static_assert(
                    std::is_same_v<
                        T, Dynamic_rigid_body_dynamic_rigid_body_contact *>);
                resolve_dynamic_rigid_body_dynamic_rigid_body_contact_velocity(
                    arg, max_separating_velocity_for_bounce);
              }
            },
            *contact);
      } else {
        break;
      }
    }
  }

  std::optional<std::variant<Particle_particle_contact *,
                             Particle_static_rigid_body_contact *,
                             Particle_dynamic_rigid_body_contact *,
                             Static_rigid_body_dynamic_rigid_body_contact *,
                             Dynamic_rigid_body_dynamic_rigid_body_contact *>>
  find_contact_of_least_negative_separating_velocity() {
    auto retval = std::optional<std::variant<
        Particle_particle_contact *, Particle_static_rigid_body_contact *,
        Particle_dynamic_rigid_body_contact *,
        Static_rigid_body_dynamic_rigid_body_contact *,
        Dynamic_rigid_body_dynamic_rigid_body_contact *>>{};
    auto retval_separating_velocity = 0.0f;
    for (auto &contact : _particle_particle_contacts) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    }
    for (auto &contact : _particle_static_rigid_body_contacts) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    }
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    }
    for (auto &contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    }
    for (auto &contact : _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    }
    return retval;
  }

  void resolve_particle_particle_contact_velocity(
      Particle_particle_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const particles = contact->particles;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const separating_velocity_per_impulse =
        particles[0]->inverse_mass + particles[1]->inverse_mass;
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particles[0]->material.restitution_coefficient +
                      particles[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const particle_velocity_deltas =
        std::array<math::Vec3f, 2>{impulse * particles[0]->inverse_mass,
                                   impulse * -particles[1]->inverse_mass};
    particles[0]->velocity += particle_velocity_deltas[0];
    particles[1]->velocity += particle_velocity_deltas[1];
    contact->separating_velocity += delta_separating_velocity;
    for (auto &other_contact : _particle_particle_contacts) {
      if (&other_contact != contact) {
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.particles[0] == particles[i]) {
            other_contact.separating_velocity +=
                math::dot(particle_velocity_deltas[i], other_contact.normal);
            break;
          }
        }
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.particles[1] == particles[i]) {
            other_contact.separating_velocity -=
                math::dot(particle_velocity_deltas[i], other_contact.normal);
            break;
          }
        }
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.particle == particles[i]) {
          other_contact.separating_velocity +=
              math::dot(particle_velocity_deltas[i], other_contact.normal);
          break;
        }
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.particle == particles[i]) {
          other_contact.separating_velocity +=
              math::dot(particle_velocity_deltas[i], other_contact.normal);
          break;
        }
      }
    }
  }

  void resolve_particle_static_rigid_body_contact_velocity(
      Particle_static_rigid_body_contact *contact,
      math::Vec3f const &gravitational_velocity_delta,
      float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const body = contact->body;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle->material.restitution_coefficient +
                      body->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity -
        std::min(separating_velocity -
                     math::dot(gravitational_velocity_delta, normal),
                 0.0f) *
            restitution_coefficient;
    auto const particle_velocity_delta = delta_separating_velocity * normal;
    particle->velocity += particle_velocity_delta;
    contact->separating_velocity += delta_separating_velocity;
    for (auto &other_contact : _particle_particle_contacts) {
      if (other_contact.particles[0] == particle) {
        other_contact.separating_velocity +=
            math::dot(particle_velocity_delta, other_contact.normal);
      } else if (other_contact.particles[1] == particle) {
        other_contact.separating_velocity -=
            math::dot(particle_velocity_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      if (&other_contact != contact) {
        if (other_contact.particle == particle) {
          other_contact.separating_velocity +=
              math::dot(particle_velocity_delta, other_contact.normal);
        }
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (other_contact.particle == particle) {
        other_contact.separating_velocity +=
            math::dot(particle_velocity_delta, other_contact.normal);
      }
    }
  }

  void resolve_particle_dynamic_rigid_body_contact_velocity(
      Particle_dynamic_rigid_body_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const body = contact->body;
    auto const body_relative_contact_position = contact->body_relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const body_rotation = math::Mat3x3f::rotation(body->orientation);
    auto const body_inverse_rotation = math::transpose(body_rotation);
    auto const body_inverse_inertia_tensor =
        body_rotation * body->inverse_inertia_tensor * body_inverse_rotation;
    auto const body_angular_impulse_per_impulse =
        math::cross(body_relative_contact_position, normal);
    auto const body_angular_velocity_per_impulse =
        body_inverse_inertia_tensor * body_angular_impulse_per_impulse;
    auto const separating_velocity_per_impulse =
        particle->inverse_mass + body->inverse_mass +
        math::dot(math::cross(body_angular_velocity_per_impulse,
                              body_relative_contact_position),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle->material.restitution_coefficient +
                      body->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const particle_velocity_delta = impulse * particle->inverse_mass;
    auto const body_velocity_delta = -impulse * body->inverse_mass;
    auto const body_angular_velocity_delta =
        body_inverse_inertia_tensor *
        math::cross(body_relative_contact_position, -impulse);
    particle->velocity += particle_velocity_delta;
    body->velocity += body_velocity_delta;
    body->angular_velocity += body_angular_velocity_delta;
    contact->separating_velocity += delta_separating_velocity;
    for (auto &other_contact : _particle_particle_contacts) {
      if (other_contact.particles[0] == particle) {
        other_contact.separating_velocity +=
            math::dot(particle_velocity_delta, other_contact.normal);
      } else if (other_contact.particles[1] == particle) {
        other_contact.separating_velocity -=
            math::dot(particle_velocity_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_static_rigid_body_contacts) {
      if (other_contact.particle == particle) {
        other_contact.separating_velocity +=
            math::dot(particle_velocity_delta, other_contact.normal);
      }
    }
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (&other_contact != contact) {
        if (other_contact.particle == particle) {
          other_contact.separating_velocity +=
              math::dot(particle_velocity_delta, other_contact.normal);
        } else if (other_contact.body == body) {
          other_contact.separating_velocity -=
              math::dot(body_velocity_delta +
                            math::cross(body_angular_velocity_delta,
                                        other_contact.body_relative_position),
                        other_contact.normal);
        }
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.dynamic_body == body) {
        other_contact.separating_velocity -= math::dot(
            body_velocity_delta +
                math::cross(body_angular_velocity_delta,
                            other_contact.dynamic_body_relative_position),
            other_contact.normal);
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.bodies[0] == body) {
        other_contact.separating_velocity +=
            math::dot(body_velocity_delta +
                          math::cross(body_angular_velocity_delta,
                                      other_contact.body_relative_positions[0]),
                      other_contact.normal);
      } else if (other_contact.bodies[1] == body) {
        other_contact.separating_velocity -=
            math::dot(body_velocity_delta +
                          math::cross(body_angular_velocity_delta,
                                      other_contact.body_relative_positions[1]),
                      other_contact.normal);
      }
    }
  }

  void resolve_static_rigid_body_dynamic_rigid_body_contact_velocity(
      Static_rigid_body_dynamic_rigid_body_contact *contact,
      math::Vec3f const &gravitational_velocity_delta,
      float max_separating_velocity_for_bounce) {
    auto const static_body = contact->static_body;
    auto const dynamic_body = contact->dynamic_body;
    auto const dynamic_body_relative_contact_position =
        contact->dynamic_body_relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const dynamic_body_rotation =
        math::Mat3x3f::rotation(dynamic_body->orientation);
    auto const dynamic_body_inverse_rotation =
        math::transpose(dynamic_body_rotation);
    auto const dynamic_body_inverse_inertia_tensor =
        dynamic_body_rotation * dynamic_body->inverse_inertia_tensor *
        dynamic_body_inverse_rotation;
    auto const angular_impulse_per_impulse =
        math::cross(dynamic_body_relative_contact_position, normal);
    auto const angular_velocity_per_impulse =
        dynamic_body_inverse_inertia_tensor * angular_impulse_per_impulse;
    auto const separating_velocity_per_impulse =
        dynamic_body->inverse_mass +
        math::dot(math::cross(angular_velocity_per_impulse,
                              dynamic_body_relative_contact_position),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (static_body->material.restitution_coefficient +
                      dynamic_body->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity -
        std::min(separating_velocity -
                     math::dot(gravitational_velocity_delta, normal),
                 0.0f) *
            restitution_coefficient;
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const dynamic_body_velocity_delta =
        -impulse * dynamic_body->inverse_mass;
    auto const dynamic_body_angular_velocity_delta =
        dynamic_body_inverse_inertia_tensor *
        math::cross(dynamic_body_relative_contact_position, -impulse);
    dynamic_body->velocity += dynamic_body_velocity_delta;
    dynamic_body->angular_velocity += dynamic_body_angular_velocity_delta;
    contact->separating_velocity += delta_separating_velocity;
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      if (other_contact.body == dynamic_body) {
        other_contact.separating_velocity -=
            math::dot(dynamic_body_velocity_delta +
                          math::cross(dynamic_body_angular_velocity_delta,
                                      other_contact.body_relative_position),
                      other_contact.normal);
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.dynamic_body == dynamic_body) {
        other_contact.separating_velocity -= math::dot(
            dynamic_body_velocity_delta +
                math::cross(dynamic_body_angular_velocity_delta,
                            other_contact.dynamic_body_relative_position),
            other_contact.normal);
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (other_contact.bodies[0] == dynamic_body) {
        other_contact.separating_velocity +=
            math::dot(dynamic_body_velocity_delta +
                          math::cross(dynamic_body_angular_velocity_delta,
                                      other_contact.body_relative_positions[0]),
                      other_contact.normal);
      } else if (other_contact.bodies[1] == dynamic_body) {
        other_contact.separating_velocity -=
            math::dot(dynamic_body_velocity_delta +
                          math::cross(dynamic_body_angular_velocity_delta,
                                      other_contact.body_relative_positions[1]),
                      other_contact.normal);
      }
    }
  }

  void resolve_dynamic_rigid_body_dynamic_rigid_body_contact_velocity(
      Dynamic_rigid_body_dynamic_rigid_body_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const bodies = contact->bodies;
    auto const body_relative_contact_positions =
        contact->body_relative_positions;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const body_rotations = std::array<math::Mat3x3f, 2>{
        math::Mat3x3f::rotation(bodies[0]->orientation),
        math::Mat3x3f::rotation(bodies[1]->orientation)};
    auto const body_inverse_rotations = std::array<math::Mat3x3f, 2>{
        math::transpose(body_rotations[0]), math::transpose(body_rotations[1])};
    auto const body_inverse_inertia_tensors = std::array<math::Mat3x3f, 2>{
        body_rotations[0] * bodies[0]->inverse_inertia_tensor *
            body_inverse_rotations[0],
        body_rotations[1] * bodies[1]->inverse_inertia_tensor *
            body_inverse_rotations[1]};
    auto const body_angular_impulses_per_impulse = std::array<math::Vec3f, 2>{
        math::cross(body_relative_contact_positions[0], normal),
        math::cross(body_relative_contact_positions[1], normal)};
    auto const body_angular_velocities_per_impulse = std::array<math::Vec3f, 2>{
        body_inverse_inertia_tensors[0] * body_angular_impulses_per_impulse[0],
        body_inverse_inertia_tensors[1] * body_angular_impulses_per_impulse[1]};
    auto const separating_velocity_per_impulse =
        bodies[0]->inverse_mass + bodies[1]->inverse_mass +
        math::dot(math::cross(body_angular_velocities_per_impulse[0],
                              body_relative_contact_positions[0]) +
                      math::cross(body_angular_velocities_per_impulse[1],
                                  body_relative_contact_positions[1]),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (bodies[0]->material.restitution_coefficient +
                      bodies[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const body_velocity_deltas = std::array<math::Vec3f, 2>{
        impulse * bodies[0]->inverse_mass, -impulse * bodies[1]->inverse_mass};
    auto const body_angular_velocity_deltas = std::array<math::Vec3f, 2>{
        body_inverse_inertia_tensors[0] *
            math::cross(body_relative_contact_positions[0], impulse),
        body_inverse_inertia_tensors[1] *
            math::cross(body_relative_contact_positions[1], -impulse)};
    bodies[0]->velocity += body_velocity_deltas[0];
    bodies[0]->angular_velocity += body_angular_velocity_deltas[0];
    bodies[1]->velocity += body_velocity_deltas[1];
    bodies[1]->angular_velocity += body_angular_velocity_deltas[1];
    contact->separating_velocity += delta_separating_velocity;
    for (auto &other_contact : _particle_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.body == bodies[i]) {
          other_contact.separating_velocity -=
              math::dot(body_velocity_deltas[i] +
                            math::cross(body_angular_velocity_deltas[i],
                                        other_contact.body_relative_position),
                        other_contact.normal);
          break;
        }
      }
    }
    for (auto &other_contact : _static_rigid_body_dynamic_rigid_body_contacts) {
      for (auto i = 0; i != 2; ++i) {
        if (other_contact.dynamic_body == bodies[i]) {
          other_contact.separating_velocity -= math::dot(
              body_velocity_deltas[i] +
                  math::cross(body_angular_velocity_deltas[i],
                              other_contact.dynamic_body_relative_position),
              other_contact.normal);
          break;
        }
      }
    }
    for (auto &other_contact :
         _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      if (&other_contact != contact) {
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.bodies[0] == bodies[i]) {
            other_contact.separating_velocity += math::dot(
                body_velocity_deltas[i] +
                    math::cross(body_angular_velocity_deltas[i],
                                other_contact.body_relative_positions[0]),
                other_contact.normal);
            break;
          }
        }
        for (auto i = 0; i != 2; ++i) {
          if (other_contact.bodies[1] == bodies[i]) {
            other_contact.separating_velocity -= math::dot(
                body_velocity_deltas[i] +
                    math::cross(body_angular_velocity_deltas[i],
                                other_contact.body_relative_positions[1]),
                other_contact.normal);
            break;
          }
        }
      }
    }
  }

  void call_particle_motion_callbacks() {
    _particles.for_each([&](Particle_handle handle, Particle_data *data) {
      if (data->motion_callback != nullptr) {
        data->motion_callback->on_particle_motion({handle, data->position});
      }
    });
  }

  void call_dynamic_rigid_body_motion_callbacks() {
    _dynamic_rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle handle, Dynamic_rigid_body_data *data) {
          if (data->motion_callback != nullptr) {
            data->motion_callback->on_dynamic_rigid_body_motion(
                {handle, data->position, data->orientation});
          }
        });
  }

  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  Particle_storage _particles;
  Static_rigid_body_storage _static_rigid_bodies;
  Dynamic_rigid_body_storage _dynamic_rigid_bodies;
  std::vector<std::pair<Particle_handle, Particle_handle>>
      _potential_particle_particle_contacts;
  std::vector<std::pair<Particle_handle, Static_rigid_body_handle>>
      _potential_particle_static_rigid_body_contacts;
  std::vector<std::pair<Particle_handle, Dynamic_rigid_body_handle>>
      _potential_particle_dynamic_rigid_body_contacts;
  std::vector<std::pair<Static_rigid_body_handle, Dynamic_rigid_body_handle>>
      _potential_static_rigid_body_dynamic_rigid_body_contacts;
  std::vector<std::pair<Dynamic_rigid_body_handle, Dynamic_rigid_body_handle>>
      _potential_dynamic_rigid_body_dynamic_rigid_body_contacts;
  std::vector<Particle_particle_contact> _particle_particle_contacts;
  std::vector<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_contacts;
  std::vector<Particle_dynamic_rigid_body_contact>
      _particle_dynamic_rigid_body_contacts;
  std::vector<Static_rigid_body_dynamic_rigid_body_contact>
      _static_rigid_body_dynamic_rigid_body_contacts;
  std::vector<Dynamic_rigid_body_dynamic_rigid_body_contact>
      _dynamic_rigid_body_dynamic_rigid_body_contacts;
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