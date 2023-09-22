#include "space.h"

#include <cstdint>

#include <iostream>
#include <random>

#include <ankerl/unordered_dense.h>

#include "../util/contiguous_storage.h"
#include "aabb_tree.h"

namespace marlon {
namespace physics {
namespace {
using Aabb_tree_payload_t =
    std::variant<Particle_handle, Static_rigid_body_handle,
                 Dynamic_rigid_body_handle>;

struct Particle_particle_contact {
  std::array<Particle_handle, 2> particles;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Particle_static_rigid_body_contact {
  Particle_handle particle;
  Static_rigid_body_handle body;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Particle_dynamic_rigid_body_contact {
  Particle_handle particle;
  Dynamic_rigid_body_handle body;
  math::Vec3f body_relative_position;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Dynamic_rigid_body_static_rigid_body_contact {
  Dynamic_rigid_body_handle dynamic_body;
  Static_rigid_body_handle static_body;
  math::Vec3f dynamic_body_relative_position;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Dynamic_rigid_body_dynamic_rigid_body_contact {
  std::array<Dynamic_rigid_body_handle, 2> bodies;
  std::array<math::Vec3f, 2> body_relative_positions;
  math::Vec3f normal;
  float separation;
  float separating_velocity;
};

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  Particle_particle_contact **particle_contacts;
  Particle_static_rigid_body_contact **static_rigid_body_contacts;
  Particle_dynamic_rigid_body_contact **dynamic_rigid_body_contacts;
  Particle_motion_callback *motion_callback;
  // std::uint64_t collision_flags;
  // std::uint64_t collision_mask;
  math::Vec3f position;
  math::Vec3f velocity;
  float inverse_mass;
  float radius;
  Material material;
  std::uint16_t particle_contact_count;
  std::uint16_t static_rigid_body_contact_count;
  std::uint16_t dynamic_rigid_body_contact_count;
};

struct Dynamic_rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *broadphase_node;
  Particle_dynamic_rigid_body_contact **particle_contacts;
  Dynamic_rigid_body_static_rigid_body_contact **static_rigid_body_contacts;
  Dynamic_rigid_body_dynamic_rigid_body_contact **dynamic_rigid_body_contacts;
  Dynamic_rigid_body_motion_callback *motion_callback;
  // std::uint64_t collision_flags;
  // std::uint64_t collision_mask;
  math::Vec3f position;
  math::Vec3f velocity;
  math::Quatf orientation;
  math::Vec3f angular_velocity;
  float inverse_mass;
  math::Mat3x3f inverse_inertia_tensor;
  Shape shape;
  Material material;
  std::uint16_t particle_contact_count;
  std::uint16_t static_rigid_body_contact_count;
  std::uint16_t dynamic_rigid_body_contact_count;
};

struct Static_rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  // std::uint64_t collision_flags;
  // std::uint64_t collision_mask;
  math::Mat3x4f transform;
  math::Mat3x4f inverse_transform;
  Shape shape;
  Material material;
};

class Particle_storage {
public:
  explicit Particle_storage(std::size_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size * sizeof(Particle_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
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
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n; ++i) {
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
  std::vector<std::size_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

class Dynamic_rigid_body_storage {
public:
  explicit Dynamic_rigid_body_storage(std::size_t size)
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Dynamic_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
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
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n; ++i) {
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
  std::vector<std::size_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

class Static_rigid_body_storage {
public:
  explicit Static_rigid_body_storage(std::size_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Static_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
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
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n; ++i) {
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
  std::vector<std::size_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

auto const max_tangential_move_coefficient = 0.1f;
} // namespace

class Space::Impl {
public:
  explicit Impl(Space_create_info const &create_info)
      : _particles{create_info.max_particles},
        _static_rigid_bodies{create_info.max_static_rigid_bodies},
        _dynamic_rigid_bodies{create_info.max_dynamic_rigid_bodies},
        _potential_particle_particle_contacts{
            create_info.max_particle_particle_contacts},
        _potential_particle_dynamic_rigid_body_contacts{
            create_info.max_particle_dynamic_rigid_body_contacts},
        _potential_particle_static_rigid_body_contacts{
            create_info.max_particle_static_rigid_body_contacts},
        _potential_dynamic_rigid_body_dynamic_rigid_body_contacts{
            create_info.max_dynamic_rigid_body_dynamic_rigid_body_contacts},
        _potential_dynamic_rigid_body_static_rigid_body_contacts{
            create_info.max_dynamic_rigid_body_static_rigid_body_contacts},
        _particle_particle_contacts{create_info.max_particle_particle_contacts},
        _particle_dynamic_rigid_body_contacts{
            create_info.max_particle_dynamic_rigid_body_contacts},
        _particle_static_rigid_body_contacts{
            create_info.max_particle_static_rigid_body_contacts},
        _dynamic_rigid_body_dynamic_rigid_body_contacts{
            create_info.max_dynamic_rigid_body_dynamic_rigid_body_contacts},
        _dynamic_rigid_body_static_rigid_body_contacts{
            create_info.max_dynamic_rigid_body_static_rigid_body_contacts},
        _particle_particle_contact_pointers{
            2 * create_info.max_particle_particle_contacts},
        _particle_dynamic_rigid_body_contact_pointers{
            2 * create_info.max_particle_dynamic_rigid_body_contacts},
        _particle_static_rigid_body_contact_pointers{
            create_info.max_particle_static_rigid_body_contacts},
        _dynamic_rigid_body_dynamic_rigid_body_contact_pointers{
            2 * create_info.max_dynamic_rigid_body_dynamic_rigid_body_contacts},
        _dynamic_rigid_body_static_rigid_body_contact_pointers{
            create_info.max_dynamic_rigid_body_static_rigid_body_contacts},
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
                      // .collision_flags = create_info.collision_flags,
                      // .collision_mask = create_info.collision_mask,
                      .position = create_info.position,
                      .velocity = create_info.velocity,
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
        // .collision_flags = create_info.collision_flags,
        // .collision_mask = create_info.collision_mask,
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
        .broadphase_node = _aabb_tree.create_leaf(bounds, handle),
        .motion_callback = create_info.motion_callback,
        // .collision_flags = create_info.collision_flags,
        // .collision_mask = create_info.collision_mask,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
        .inverse_mass = 1.0f / create_info.mass,
        .inverse_inertia_tensor = inverse(create_info.inertia_tensor),
        .shape = create_info.shape,
        .material = create_info.material};
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
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate(h);
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
    _potential_dynamic_rigid_body_static_rigid_body_contacts.clear();
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
                      _potential_dynamic_rigid_body_static_rigid_body_contacts
                          .push_back({second_handle, first_handle});
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
                      _potential_dynamic_rigid_body_static_rigid_body_contacts
                          .push_back({first_handle, second_handle});
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

  void integrate(float h) {
    integrate_particles(h);
    integrate_dynamic_rigid_bodies(h);
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
    reset_particle_contact_counts();
    reset_dynamic_rigid_body_contact_counts();
    find_and_count_particle_particle_contacts();
    find_and_count_particle_static_rigid_body_contacts();
    find_and_count_particle_dynamic_rigid_body_contacts();
    find_and_count_dynamic_rigid_body_static_rigid_body_contacts();
    find_and_count_dynamic_rigid_body_dynamic_rigid_body_contacts();
    reset_contact_pointer_storage();
    allocate_particle_contact_pointers();
    allocate_dynamic_rigid_body_contact_pointers();
    reset_particle_contact_counts();
    reset_dynamic_rigid_body_contact_counts();
    assign_and_count_particle_particle_contact_pointers();
    assign_and_count_particle_dynamic_rigid_body_contact_pointers();
    assign_and_count_particle_static_rigid_body_contact_pointers();
    assign_and_count_dynamic_rigid_body_dynamic_rigid_body_contact_pointers();
    assign_and_count_dynamic_rigid_body_static_rigid_body_contact_pointers();
  }

  void reset_particle_contact_counts() {
    _particles.for_each([](Particle_handle, Particle_data *data) {
      data->particle_contact_count = 0;
      data->dynamic_rigid_body_contact_count = 0;
      data->static_rigid_body_contact_count = 0;
    });
  }

  void reset_dynamic_rigid_body_contact_counts() {
    _dynamic_rigid_bodies.for_each(
        [](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->particle_contact_count = 0;
          data->dynamic_rigid_body_contact_count = 0;
          data->static_rigid_body_contact_count = 0;
        });
  }

  void find_and_count_particle_particle_contacts() {
    _particle_particle_contacts.clear();
    for (auto const &particle_handles : _potential_particle_particle_contacts) {
      auto const particle_datas = std::array<Particle_data *, 2>{
          _particles.data(particle_handles.first),
          _particles.data(particle_handles.second)};
      auto const displacement =
          particle_datas[0]->position - particle_datas[1]->position;
      auto const distance2 = math::length_squared(displacement);
      auto const contact_distance =
          particle_datas[0]->radius + particle_datas[1]->radius;
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
        auto const separating_velocity = math::dot(
            particle_datas[0]->velocity - particle_datas[1]->velocity, normal);
        _particle_particle_contacts.push_back(
            {.particles = {particle_handles.first, particle_handles.second},
             .normal = normal,
             .separation = separation,
             .separating_velocity = separating_velocity});
        for (auto i = 0; i != 2; ++i) {
          ++particle_datas[i]->particle_contact_count;
        }
      }
    }
  }

  void find_and_count_particle_dynamic_rigid_body_contacts() {
    _particle_dynamic_rigid_body_contacts.clear();
    for (auto [particle_handle, body_handle] :
         _potential_particle_dynamic_rigid_body_contacts) {
      auto const particle_data = _particles.data(particle_handle);
      auto const body_data = _dynamic_rigid_bodies.data(body_handle);
      auto const body_transform =
          math::Mat3x4f::rigid(body_data->position, body_data->orientation);
      auto const inverse_body_transform = math::rigid_inverse(body_transform);
      if (auto const contact_geometry =
              particle_shape_positioned_contact_geometry(
                  particle_data->position, particle_data->radius,
                  body_data->shape, body_transform, inverse_body_transform)) {
        auto const body_relative_position =
            contact_geometry->position - body_data->position;
        auto const relative_velocity =
            particle_data->velocity -
            (body_data->velocity +
             math::cross(body_data->angular_velocity, body_relative_position));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _particle_dynamic_rigid_body_contacts.push_back(
            {.particle = particle_handle,
             .body = body_handle,
             .body_relative_position = body_relative_position,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
        ++particle_data->dynamic_rigid_body_contact_count;
        ++body_data->particle_contact_count;
      }
    }
  }

  void find_and_count_particle_static_rigid_body_contacts() {
    _particle_static_rigid_body_contacts.clear();
    for (auto [particle_handle, body_handle] :
         _potential_particle_static_rigid_body_contacts) {
      auto const particle_data = _particles.data(particle_handle);
      auto const body_data = _static_rigid_bodies.data(body_handle);
      if (auto const contact_geometry =
              particle_shape_positionless_contact_geometry(
                  particle_data->position, particle_data->radius,
                  body_data->shape, body_data->transform,
                  body_data->inverse_transform)) {
        auto const separating_velocity =
            math::dot(particle_data->velocity, contact_geometry->normal);
        _particle_static_rigid_body_contacts.push_back(
            {.particle = particle_handle,
             .body = body_handle,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
        ++particle_data->static_rigid_body_contact_count;
      }
    }
  }

  void find_and_count_dynamic_rigid_body_dynamic_rigid_body_contacts() {
    _dynamic_rigid_body_dynamic_rigid_body_contacts.clear();
    for (auto const &body_handles :
         _potential_dynamic_rigid_body_dynamic_rigid_body_contacts) {
      auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
          _dynamic_rigid_bodies.data(body_handles.first),
          _dynamic_rigid_bodies.data(body_handles.second)};
      auto const body_transforms = std::array<math::Mat3x4f, 2>{
          math::Mat3x4f::rigid(body_datas[0]->position,
                               body_datas[0]->orientation),
          math::Mat3x4f::rigid(body_datas[1]->position,
                               body_datas[1]->orientation)};
      auto const body_inverse_transforms =
          std::array<math::Mat3x4f, 2>{math::rigid_inverse(body_transforms[0]),
                                       math::rigid_inverse(body_transforms[1])};
      if (auto const contact_geometry = shape_shape_contact_geometry(
              body_datas[0]->shape, body_transforms[0],
              body_inverse_transforms[0], body_datas[1]->shape,
              body_transforms[1], body_inverse_transforms[1])) {
        auto const body_relative_contact_positions = std::array<math::Vec3f, 2>{
            contact_geometry->position - body_datas[0]->position,
            contact_geometry->position - body_datas[1]->position};
        auto const relative_velocity =
            (body_datas[0]->velocity +
             math::cross(body_datas[0]->angular_velocity,
                         body_relative_contact_positions[0])) -
            (body_datas[1]->velocity +
             math::cross(body_datas[1]->angular_velocity,
                         body_relative_contact_positions[1]));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _dynamic_rigid_body_dynamic_rigid_body_contacts.push_back(
            {.bodies = {body_handles.first, body_handles.second},
             .body_relative_positions = body_relative_contact_positions,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
        for (auto i = 0; i != 2; ++i) {
          ++body_datas[i]->dynamic_rigid_body_contact_count;
        }
      }
    }
  }

  void find_and_count_dynamic_rigid_body_static_rigid_body_contacts() {
    _dynamic_rigid_body_static_rigid_body_contacts.clear();
    for (auto [dynamic_body_handle, static_body_handle] :
         _potential_dynamic_rigid_body_static_rigid_body_contacts) {
      auto const dynamic_body_data =
          _dynamic_rigid_bodies.data(dynamic_body_handle);
      auto const dynamic_body_transform = math::Mat3x4f::rigid(
          dynamic_body_data->position, dynamic_body_data->orientation);
      auto const dynamic_body_inverse_transform =
          math::rigid_inverse(dynamic_body_transform);
      auto const static_body_data =
          _static_rigid_bodies.data(static_body_handle);
      if (auto const contact_geometry = shape_shape_contact_geometry(
              dynamic_body_data->shape, dynamic_body_transform,
              dynamic_body_inverse_transform, static_body_data->shape,
              static_body_data->transform,
              static_body_data->inverse_transform)) {
        auto const dynamic_body_relative_contact_position =
            contact_geometry->position - dynamic_body_data->position;
        auto const relative_velocity =
            dynamic_body_data->velocity +
            math::cross(dynamic_body_data->angular_velocity,
                        dynamic_body_relative_contact_position);
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _dynamic_rigid_body_static_rigid_body_contacts.push_back(
            {.dynamic_body = dynamic_body_handle,
             .static_body = static_body_handle,
             .dynamic_body_relative_position =
                 dynamic_body_relative_contact_position,
             .normal = contact_geometry->normal,
             .separation = contact_geometry->separation,
             .separating_velocity = separating_velocity});
        ++dynamic_body_data->static_rigid_body_contact_count;
      }
    }
  }

  void reset_contact_pointer_storage() {
    _particle_particle_contact_pointers.clear();
    _particle_dynamic_rigid_body_contact_pointers.clear();
    _particle_static_rigid_body_contact_pointers.clear();
    _dynamic_rigid_body_dynamic_rigid_body_contact_pointers.clear();
    _dynamic_rigid_body_static_rigid_body_contact_pointers.clear();
  }

  void allocate_particle_contact_pointers() {
    _particles.for_each([this](Particle_handle, Particle_data *data) {
      data->particle_contacts = _particle_particle_contact_pointers.end();
      data->dynamic_rigid_body_contacts =
          _particle_dynamic_rigid_body_contact_pointers.end();
      data->static_rigid_body_contacts =
          _particle_static_rigid_body_contact_pointers.end();
      _particle_particle_contact_pointers.push_back_n(
          data->particle_contact_count, {});
      _particle_dynamic_rigid_body_contact_pointers.push_back_n(
          data->dynamic_rigid_body_contact_count, {});
      _particle_static_rigid_body_contact_pointers.push_back_n(
          data->static_rigid_body_contact_count, {});
    });
  }

  void allocate_dynamic_rigid_body_contact_pointers() {
    _dynamic_rigid_bodies.for_each(
        [this](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->particle_contacts =
              _particle_dynamic_rigid_body_contact_pointers.end();
          data->dynamic_rigid_body_contacts =
              _dynamic_rigid_body_dynamic_rigid_body_contact_pointers.end();
          data->static_rigid_body_contacts =
              _dynamic_rigid_body_static_rigid_body_contact_pointers.end();
          _particle_dynamic_rigid_body_contact_pointers.push_back_n(
              data->particle_contact_count, {});
          _dynamic_rigid_body_dynamic_rigid_body_contact_pointers.push_back_n(
              data->dynamic_rigid_body_contact_count, {});
          _dynamic_rigid_body_static_rigid_body_contact_pointers.push_back_n(
              data->static_rigid_body_contact_count, {});
        });
  }

  void assign_and_count_particle_particle_contact_pointers() {
    for (auto &contact : _particle_particle_contacts) {
      auto const particle_datas =
          std::array<Particle_data *, 2>{_particles.data(contact.particles[0]),
                                         _particles.data(contact.particles[1])};
      for (auto i = 0; i != 2; ++i) {
        particle_datas[i]
            ->particle_contacts[particle_datas[i]->particle_contact_count++] =
            &contact;
      }
    }
  }

  void assign_and_count_particle_dynamic_rigid_body_contact_pointers() {
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      auto const dynamic_rigid_body_data =
          _dynamic_rigid_bodies.data(contact.body);
      particle_data->dynamic_rigid_body_contacts
          [particle_data->dynamic_rigid_body_contact_count++] = &contact;
      dynamic_rigid_body_data->particle_contacts
          [dynamic_rigid_body_data->particle_contact_count++] = &contact;
    }
  }

  void assign_and_count_particle_static_rigid_body_contact_pointers() {
    for (auto &contact : _particle_static_rigid_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      particle_data->static_rigid_body_contacts
          [particle_data->static_rigid_body_contact_count++] = &contact;
    }
  }

  void
  assign_and_count_dynamic_rigid_body_dynamic_rigid_body_contact_pointers() {
    for (auto &contact : _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
          _dynamic_rigid_bodies.data(contact.bodies[0]),
          _dynamic_rigid_bodies.data(contact.bodies[1])};
      for (auto i = 0; i != 2; ++i) {
        body_datas[i]->dynamic_rigid_body_contacts
            [body_datas[i]->dynamic_rigid_body_contact_count++] = &contact;
      }
    }
  }

  void
  assign_and_count_dynamic_rigid_body_static_rigid_body_contact_pointers() {
    for (auto &contact : _dynamic_rigid_body_static_rigid_body_contacts) {
      auto const dynamic_body_data =
          _dynamic_rigid_bodies.data(contact.dynamic_body);
      dynamic_body_data->static_rigid_body_contacts
          [dynamic_body_data->static_rigid_body_contact_count++] = &contact;
    }
  }

  int count_contacts() const {
    return static_cast<int>(
        _particle_particle_contacts.size() +
        _particle_static_rigid_body_contacts.size() +
        _particle_dynamic_rigid_body_contacts.size() +
        _dynamic_rigid_body_static_rigid_body_contacts.size() +
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
                      T, Dynamic_rigid_body_static_rigid_body_contact *>) {
                resolve_dynamic_rigid_body_static_rigid_body_contact_position(
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
  }

  std::optional<std::variant<Particle_particle_contact *,
                             Particle_static_rigid_body_contact *,
                             Particle_dynamic_rigid_body_contact *,
                             Dynamic_rigid_body_static_rigid_body_contact *,
                             Dynamic_rigid_body_dynamic_rigid_body_contact *>>
  find_contact_of_least_negative_separation() {
    auto retval = std::optional<std::variant<
        Particle_particle_contact *, Particle_static_rigid_body_contact *,
        Particle_dynamic_rigid_body_contact *,
        Dynamic_rigid_body_static_rigid_body_contact *,
        Dynamic_rigid_body_dynamic_rigid_body_contact *>>{};
    auto retval_separation = 0.0f;
    auto const process_contact = [&](auto &contact) {
      if (contact.separation < retval_separation) {
        retval = &contact;
        retval_separation = contact.separation;
      }
    };
    for (auto &contact : _particle_particle_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _particle_static_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _dynamic_rigid_body_static_rigid_body_contacts) {
      process_contact(contact);
    }
    return retval;
  }

  void resolve_particle_particle_contact_position(
      Particle_particle_contact *contact) {
    auto const particles = contact->particles;
    auto const particle_datas = std::array<Particle_data *, 2>{
        _particles.data(particles[0]), _particles.data(particles[1])};
    auto const distance_per_impulse =
        particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
    auto const impulse_per_distance = 1.0f / distance_per_impulse;
    auto const impulse =
        -contact->separation * impulse_per_distance * contact->normal;
    auto const particle_position_deltas =
        std::array<math::Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
                                   impulse * -particle_datas[1]->inverse_mass};
    particle_datas[0]->position += particle_position_deltas[0];
    particle_datas[1]->position += particle_position_deltas[1];
    for (auto i = 0; i != 2; ++i) {
      update_particle_contact_separations(particles[i],
                                          particle_position_deltas[i]);
    }
  }

  void resolve_particle_dynamic_rigid_body_contact_position(
      Particle_dynamic_rigid_body_contact *contact) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _dynamic_rigid_bodies.data(body);
    auto const body_rotation = math::Mat3x3f::rotation(body_data->orientation);
    auto const body_inverse_rotation = math::transpose(body_rotation);
    auto const body_inverse_inertia_tensor = body_rotation *
                                             body_data->inverse_inertia_tensor *
                                             body_inverse_rotation;
    auto const body_relative_contact_position = contact->body_relative_position;
    auto const body_angular_impulse_per_impulse =
        math::cross(body_relative_contact_position, contact->normal);
    auto const body_angular_velocity_per_impulse =
        body_inverse_inertia_tensor * body_angular_impulse_per_impulse;
    auto const body_tangential_velocity_per_impulse = math::cross(
        body_angular_velocity_per_impulse, body_relative_contact_position);
    auto const particle_separating_linear_velocity_per_impulse =
        particle_data->inverse_mass;
    auto const body_separating_linear_velocity_per_impulse =
        body_data->inverse_mass;
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
    particle_data->position += particle_position_delta;
    body_data->position += body_position_delta;
    body_data->orientation += 0.5f * math::Quatf{0.0f, body_orientation_delta} *
                              body_data->orientation;
    update_particle_contact_separations(particle, particle_position_delta);
    update_dynamic_rigid_body_contact_separations(body, body_position_delta,
                                                  body_orientation_delta);
  }

  void resolve_particle_static_rigid_body_contact_position(
      Particle_static_rigid_body_contact *contact) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const particle_position_delta = contact->normal * -contact->separation;
    particle_data->position += particle_position_delta;
    update_particle_contact_separations(particle, particle_position_delta);
  }

  void resolve_dynamic_rigid_body_dynamic_rigid_body_contact_position(
      Dynamic_rigid_body_dynamic_rigid_body_contact *contact) {
    auto const bodies = contact->bodies;
    auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
        _dynamic_rigid_bodies.data(bodies[0]),
        _dynamic_rigid_bodies.data(bodies[1])};
    auto const body_rotations = std::array<math::Mat3x3f, 2>{
        math::Mat3x3f::rotation(body_datas[0]->orientation),
        math::Mat3x3f::rotation(body_datas[1]->orientation)};
    auto const body_inverse_rotations = std::array<math::Mat3x3f, 2>{
        math::transpose(body_rotations[0]), math::transpose(body_rotations[1])};
    auto const body_inverse_inertia_tensors = std::array<math::Mat3x3f, 2>{
        body_rotations[0] * body_datas[0]->inverse_inertia_tensor *
            body_inverse_rotations[0],
        body_rotations[1] * body_datas[1]->inverse_inertia_tensor *
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
        std::array<float, 2>{body_datas[0]->inverse_mass,
                             body_datas[1]->inverse_mass};
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
      body_datas[i]->position += body_position_deltas[i];
      body_datas[i]->orientation +=
          0.5f * math::Quatf{0.0f, body_orientation_deltas[i]} *
          body_datas[i]->orientation;
      update_dynamic_rigid_body_contact_separations(
          bodies[i], body_position_deltas[i], body_orientation_deltas[i]);
    }
  }

  void resolve_dynamic_rigid_body_static_rigid_body_contact_position(
      Dynamic_rigid_body_static_rigid_body_contact *contact) {
    auto const dynamic_body = contact->dynamic_body;
    auto const dynamic_body_data = _dynamic_rigid_bodies.data(dynamic_body);
    auto const dynamic_body_rotation =
        math::Mat3x3f::rotation(dynamic_body_data->orientation);
    auto const dynamic_body_inverse_rotation =
        math::transpose(dynamic_body_rotation);
    auto const dynamic_body_inverse_inertia_tensor =
        dynamic_body_rotation * dynamic_body_data->inverse_inertia_tensor *
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
        dynamic_body_data->inverse_mass;
    auto const dynamic_body_separating_tangential_velocity_per_impulse =
        math::dot(dynamic_body_tangential_velocity_per_impulse,
                  contact->normal);
    auto const separating_velocity_per_impulse =
        dynamic_body_separating_linear_velocity_per_impulse +
        dynamic_body_separating_tangential_velocity_per_impulse;
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto dynamic_body_linear_move =
        -contact->separation *
        dynamic_body_separating_linear_velocity_per_impulse *
        impulse_per_separating_velocity;
    auto dynamic_body_tangential_move =
        -contact->separation *
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
    dynamic_body_data->position += dynamic_body_position_delta;
    dynamic_body_data->orientation +=
        0.5f * math::Quatf{0.0f, dynamic_body_orientation_delta} *
        dynamic_body_data->orientation;
    update_dynamic_rigid_body_contact_separations(
        dynamic_body, dynamic_body_position_delta,
        dynamic_body_orientation_delta);
  }

  void update_particle_contact_separations(Particle_handle particle,
                                           math::Vec3f const &position_delta) {
    auto const data = _particles.data(particle);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      contact->separation +=
          (contact->particles[0] == particle ? 1.0f : -1.0f) *
          math::dot(position_delta, contact->normal);
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      contact->separation += math::dot(position_delta, contact->normal);
    }
    for (auto const contact : static_rigid_body_contacts) {
      contact->separation += math::dot(position_delta, contact->normal);
    }
  }

  void update_dynamic_rigid_body_contact_separations(
      Dynamic_rigid_body_handle body, math::Vec3f const &position_delta,
      math::Vec3f const &orientation_delta) {
    auto const data = _dynamic_rigid_bodies.data(body);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      contact->separation -= math::dot(
          position_delta +
              math::cross(orientation_delta, contact->body_relative_position),
          contact->normal);
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      if (contact->bodies[0] == body) {
        contact->separation += math::dot(
            position_delta + math::cross(orientation_delta,
                                         contact->body_relative_positions[0]),
            contact->normal);
      } else {
        contact->separation -= math::dot(
            position_delta + math::cross(orientation_delta,
                                         contact->body_relative_positions[1]),
            contact->normal);
      }
    }
    for (auto const contact : static_rigid_body_contacts) {
      contact->separation += math::dot(
          position_delta + math::cross(orientation_delta,
                                       contact->dynamic_body_relative_position),
          contact->normal);
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
                      T, Dynamic_rigid_body_static_rigid_body_contact *>) {
                resolve_dynamic_rigid_body_static_rigid_body_contact_velocity(
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
                             Dynamic_rigid_body_static_rigid_body_contact *,
                             Dynamic_rigid_body_dynamic_rigid_body_contact *>>
  find_contact_of_least_negative_separating_velocity() {
    auto retval = std::optional<std::variant<
        Particle_particle_contact *, Particle_static_rigid_body_contact *,
        Particle_dynamic_rigid_body_contact *,
        Dynamic_rigid_body_static_rigid_body_contact *,
        Dynamic_rigid_body_dynamic_rigid_body_contact *>>{};
    auto retval_separating_velocity = 0.0f;
    auto const process_contact = [&](auto &contact) {
      if (contact.separating_velocity < retval_separating_velocity) {
        retval = &contact;
        retval_separating_velocity = contact.separating_velocity;
      }
    };
    for (auto &contact : _particle_particle_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _particle_dynamic_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _particle_static_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _dynamic_rigid_body_dynamic_rigid_body_contacts) {
      process_contact(contact);
    }
    for (auto &contact : _dynamic_rigid_body_static_rigid_body_contacts) {
      process_contact(contact);
    }
    return retval;
  }

  void resolve_particle_particle_contact_velocity(
      Particle_particle_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const particles = contact->particles;
    auto const particle_datas = std::array<Particle_data *, 2>{
        _particles.data(particles[0]), _particles.data(particles[1])};
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const separating_velocity_per_impulse =
        particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_datas[0]->material.restitution_coefficient +
                      particle_datas[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const particle_velocity_deltas =
        std::array<math::Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
                                   impulse * -particle_datas[1]->inverse_mass};
    particle_datas[0]->velocity += particle_velocity_deltas[0];
    particle_datas[1]->velocity += particle_velocity_deltas[1];
    for (auto i = 0; i != 2; ++i) {
      update_particle_contact_separating_velocities(
          particles[i], particle_velocity_deltas[i]);
    }
  }

  void resolve_particle_dynamic_rigid_body_contact_velocity(
      Particle_dynamic_rigid_body_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _dynamic_rigid_bodies.data(body);
    auto const body_relative_contact_position = contact->body_relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const body_rotation = math::Mat3x3f::rotation(body_data->orientation);
    auto const body_inverse_rotation = math::transpose(body_rotation);
    auto const body_inverse_inertia_tensor = body_rotation *
                                             body_data->inverse_inertia_tensor *
                                             body_inverse_rotation;
    auto const body_angular_impulse_per_impulse =
        math::cross(body_relative_contact_position, normal);
    auto const body_angular_velocity_per_impulse =
        body_inverse_inertia_tensor * body_angular_impulse_per_impulse;
    auto const separating_velocity_per_impulse =
        particle_data->inverse_mass + body_data->inverse_mass +
        math::dot(math::cross(body_angular_velocity_per_impulse,
                              body_relative_contact_position),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_data->material.restitution_coefficient +
                      body_data->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const particle_velocity_delta = impulse * particle_data->inverse_mass;
    auto const body_velocity_delta = -impulse * body_data->inverse_mass;
    auto const body_angular_velocity_delta =
        body_inverse_inertia_tensor *
        math::cross(body_relative_contact_position, -impulse);
    particle_data->velocity += particle_velocity_delta;
    body_data->velocity += body_velocity_delta;
    body_data->angular_velocity += body_angular_velocity_delta;
    update_particle_contact_separating_velocities(particle,
                                                  particle_velocity_delta);
    update_dynamic_rigid_body_contact_separating_velocities(
        body, body_velocity_delta, body_angular_velocity_delta);
  }

  void resolve_particle_static_rigid_body_contact_velocity(
      Particle_static_rigid_body_contact *contact,
      math::Vec3f const &gravitational_velocity_delta,
      float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _static_rigid_bodies.data(body);
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_data->material.restitution_coefficient +
                      body_data->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity -
        std::min(separating_velocity -
                     math::dot(gravitational_velocity_delta, normal),
                 0.0f) *
            restitution_coefficient;
    auto const particle_velocity_delta = delta_separating_velocity * normal;
    particle_data->velocity += particle_velocity_delta;
    update_particle_contact_separating_velocities(particle,
                                                  particle_velocity_delta);
  }

  void resolve_dynamic_rigid_body_dynamic_rigid_body_contact_velocity(
      Dynamic_rigid_body_dynamic_rigid_body_contact *contact,
      float max_separating_velocity_for_bounce) {
    auto const bodies = contact->bodies;
    auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
        _dynamic_rigid_bodies.data(bodies[0]),
        _dynamic_rigid_bodies.data(bodies[1])};
    auto const body_relative_contact_positions =
        contact->body_relative_positions;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const body_rotations = std::array<math::Mat3x3f, 2>{
        math::Mat3x3f::rotation(body_datas[0]->orientation),
        math::Mat3x3f::rotation(body_datas[1]->orientation)};
    auto const body_inverse_rotations = std::array<math::Mat3x3f, 2>{
        math::transpose(body_rotations[0]), math::transpose(body_rotations[1])};
    auto const body_inverse_inertia_tensors = std::array<math::Mat3x3f, 2>{
        body_rotations[0] * body_datas[0]->inverse_inertia_tensor *
            body_inverse_rotations[0],
        body_rotations[1] * body_datas[1]->inverse_inertia_tensor *
            body_inverse_rotations[1]};
    auto const body_angular_impulses_per_impulse = std::array<math::Vec3f, 2>{
        math::cross(body_relative_contact_positions[0], normal),
        math::cross(body_relative_contact_positions[1], normal)};
    auto const body_angular_velocities_per_impulse = std::array<math::Vec3f, 2>{
        body_inverse_inertia_tensors[0] * body_angular_impulses_per_impulse[0],
        body_inverse_inertia_tensors[1] * body_angular_impulses_per_impulse[1]};
    auto const separating_velocity_per_impulse =
        body_datas[0]->inverse_mass + body_datas[1]->inverse_mass +
        math::dot(math::cross(body_angular_velocities_per_impulse[0],
                              body_relative_contact_positions[0]) +
                      math::cross(body_angular_velocities_per_impulse[1],
                                  body_relative_contact_positions[1]),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (body_datas[0]->material.restitution_coefficient +
                      body_datas[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const impulse =
        delta_separating_velocity * impulse_per_separating_velocity * normal;
    auto const body_velocity_deltas =
        std::array<math::Vec3f, 2>{impulse * body_datas[0]->inverse_mass,
                                   -impulse * body_datas[1]->inverse_mass};
    auto const body_angular_velocity_deltas = std::array<math::Vec3f, 2>{
        body_inverse_inertia_tensors[0] *
            math::cross(body_relative_contact_positions[0], impulse),
        body_inverse_inertia_tensors[1] *
            math::cross(body_relative_contact_positions[1], -impulse)};
    for (auto i = 0; i != 2; ++i) {
      body_datas[i]->velocity += body_velocity_deltas[i];
      body_datas[i]->angular_velocity += body_angular_velocity_deltas[i];
      update_dynamic_rigid_body_contact_separating_velocities(
          bodies[i], body_velocity_deltas[i], body_angular_velocity_deltas[i]);
    }
  }

  void resolve_dynamic_rigid_body_static_rigid_body_contact_velocity(
      Dynamic_rigid_body_static_rigid_body_contact *contact,
      math::Vec3f const &gravitational_velocity_delta,
      float max_separating_velocity_for_bounce) {
    auto const static_body = contact->static_body;
    auto const static_body_data = _static_rigid_bodies.data(static_body);
    auto const dynamic_body = contact->dynamic_body;
    auto const dynamic_body_data = _dynamic_rigid_bodies.data(dynamic_body);
    auto const dynamic_body_relative_contact_position =
        contact->dynamic_body_relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const dynamic_body_rotation =
        math::Mat3x3f::rotation(dynamic_body_data->orientation);
    auto const dynamic_body_inverse_rotation =
        math::transpose(dynamic_body_rotation);
    auto const dynamic_body_inverse_inertia_tensor =
        dynamic_body_rotation * dynamic_body_data->inverse_inertia_tensor *
        dynamic_body_inverse_rotation;
    auto const angular_impulse_per_impulse =
        math::cross(dynamic_body_relative_contact_position, normal);
    auto const angular_velocity_per_impulse =
        dynamic_body_inverse_inertia_tensor * angular_impulse_per_impulse;
    auto const separating_velocity_per_impulse =
        dynamic_body_data->inverse_mass +
        math::dot(math::cross(angular_velocity_per_impulse,
                              dynamic_body_relative_contact_position),
                  normal);
    auto const impulse_per_separating_velocity =
        1.0f / separating_velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (static_body_data->material.restitution_coefficient +
                      dynamic_body_data->material.restitution_coefficient)
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
        impulse * dynamic_body_data->inverse_mass;
    auto const dynamic_body_angular_velocity_delta =
        dynamic_body_inverse_inertia_tensor *
        math::cross(dynamic_body_relative_contact_position, impulse);
    dynamic_body_data->velocity += dynamic_body_velocity_delta;
    dynamic_body_data->angular_velocity += dynamic_body_angular_velocity_delta;
    update_dynamic_rigid_body_contact_separating_velocities(
        dynamic_body, dynamic_body_velocity_delta,
        dynamic_body_angular_velocity_delta);
  }

  void update_particle_contact_separating_velocities(
      Particle_handle particle, math::Vec3f const &velocity_delta) {
    auto const data = _particles.data(particle);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      contact->separating_velocity +=
          (contact->particles[0] == particle ? 1.0f : -1.0f) *
          math::dot(velocity_delta, contact->normal);
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      contact->separating_velocity +=
          math::dot(velocity_delta, contact->normal);
    }
    for (auto const contact : static_rigid_body_contacts) {
      contact->separating_velocity +=
          math::dot(velocity_delta, contact->normal);
    }
  }

  void update_dynamic_rigid_body_contact_separating_velocities(
      Dynamic_rigid_body_handle body, math::Vec3f const &velocity_delta,
      math::Vec3f const &angular_velocity_delta) {
    // other_contact.separating_velocity -=
    //     math::dot(body_velocity_delta +
    //                   math::cross(body_angular_velocity_delta,
    //                               other_contact.body_relative_position),
    //               other_contact.normal);
    auto const data = _dynamic_rigid_bodies.data(body);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      contact->separating_velocity -= math::dot(
          velocity_delta + math::cross(angular_velocity_delta,
                                       contact->body_relative_position),
          contact->normal);
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      if (contact->bodies[0] == body) {
        contact->separating_velocity += math::dot(
            velocity_delta + math::cross(angular_velocity_delta,
                                         contact->body_relative_positions[0]),
            contact->normal);
      } else {
        contact->separating_velocity -= math::dot(
            velocity_delta + math::cross(angular_velocity_delta,
                                         contact->body_relative_positions[1]),
            contact->normal);
      }
    }
    for (auto const contact : static_rigid_body_contacts) {
      contact->separating_velocity += math::dot(
          velocity_delta + math::cross(angular_velocity_delta,
                                       contact->dynamic_body_relative_position),
          contact->normal);
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
  util::Contiguous_storage<std::pair<Particle_handle, Particle_handle>>
      _potential_particle_particle_contacts;
  util::Contiguous_storage<
      std::pair<Particle_handle, Dynamic_rigid_body_handle>>
      _potential_particle_dynamic_rigid_body_contacts;
  util::Contiguous_storage<std::pair<Particle_handle, Static_rigid_body_handle>>
      _potential_particle_static_rigid_body_contacts;
  util::Contiguous_storage<
      std::pair<Dynamic_rigid_body_handle, Dynamic_rigid_body_handle>>
      _potential_dynamic_rigid_body_dynamic_rigid_body_contacts;
  util::Contiguous_storage<
      std::pair<Dynamic_rigid_body_handle, Static_rigid_body_handle>>
      _potential_dynamic_rigid_body_static_rigid_body_contacts;
  util::Contiguous_storage<Particle_particle_contact>
      _particle_particle_contacts;
  util::Contiguous_storage<Particle_dynamic_rigid_body_contact>
      _particle_dynamic_rigid_body_contacts;
  util::Contiguous_storage<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_contacts;
  util::Contiguous_storage<Dynamic_rigid_body_dynamic_rigid_body_contact>
      _dynamic_rigid_body_dynamic_rigid_body_contacts;
  util::Contiguous_storage<Dynamic_rigid_body_static_rigid_body_contact>
      _dynamic_rigid_body_static_rigid_body_contacts;
  util::Contiguous_storage<Particle_particle_contact *>
      _particle_particle_contact_pointers;
  util::Contiguous_storage<Particle_dynamic_rigid_body_contact *>
      _particle_dynamic_rigid_body_contact_pointers;
  util::Contiguous_storage<Particle_static_rigid_body_contact *>
      _particle_static_rigid_body_contact_pointers;
  util::Contiguous_storage<Dynamic_rigid_body_dynamic_rigid_body_contact *>
      _dynamic_rigid_body_dynamic_rigid_body_contact_pointers;
  util::Contiguous_storage<Dynamic_rigid_body_static_rigid_body_contact *>
      _dynamic_rigid_body_static_rigid_body_contact_pointers;
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