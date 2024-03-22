#include "space.h"

#include <cstdint>

#include <iostream>

#include "../util/array.h"
#include "../util/map.h"
#include "aabb_tree.h"

using marlon::math::Mat3x3f;
using marlon::math::Quatf;
using marlon::math::Vec3f;

namespace marlon {
namespace physics {
using util::Array;
using util::Block;
using util::Map;
using util::Stack_allocator;

using util::make_block;
namespace {
using Aabb_tree_payload_t = std::variant<Particle_handle,
                                         Static_rigid_body_handle,
                                         Dynamic_rigid_body_handle>;

enum class Object_type { particle, dynamic_rigid_body, static_rigid_body };

enum class Contact_type {
  particle_particle,
  particle_dynamic_rigid_body,
  particle_static_rigid_body,
  dynamic_rigid_body_dynamic_rigid_body,
  dynamic_rigid_body_static_rigid_body
};

struct Contact {
  math::Vec3f normal;
  float separation;
  float separating_velocity;

  Contact() = default;

  constexpr Contact(math::Vec3f const &normal,
                    float separation,
                    float separating_velocity) noexcept
      : normal{normal}, separation{separation},
        separating_velocity{separating_velocity} {}
};

struct Particle_particle_contact : Contact {
  std::array<Particle_handle, 2> particles;

  Particle_particle_contact() = default;

  constexpr Particle_particle_contact(
      math::Vec3f const &normal,
      float separation,
      float separating_velocity,
      std::array<Particle_handle, 2> const &particles) noexcept
      : Contact{normal, separation, separating_velocity}, particles{particles} {
  }
};

struct Particle_rigid_body_contact : Contact {
  Particle_handle particle;
  Dynamic_rigid_body_handle body;
  math::Vec3f relative_position;

  Particle_rigid_body_contact() = default;

  constexpr Particle_rigid_body_contact(
      math::Vec3f const &normal,
      float separation,
      float separating_velocity,
      Particle_handle particle,
      Dynamic_rigid_body_handle body,
      math::Vec3f const &body_relative_position) noexcept
      : Contact{normal, separation, separating_velocity}, particle{particle},
        body{body}, relative_position{body_relative_position} {}
};

struct Particle_static_body_contact : Contact {
  Particle_handle particle;
  Static_rigid_body_handle body;

  Particle_static_body_contact() = default;

  constexpr Particle_static_body_contact(math::Vec3f const &normal,
                                         float separation,
                                         float separating_velocity,
                                         Particle_handle particle,
                                         Static_rigid_body_handle body)
      : Contact{normal, separation, separating_velocity}, particle{particle},
        body{body} {}
};

struct Rigid_body_rigid_body_contact : Contact {
  std::array<Dynamic_rigid_body_handle, 2> bodies;
  std::array<math::Vec3f, 2> relative_positions;

  Rigid_body_rigid_body_contact() = default;

  constexpr Rigid_body_rigid_body_contact(
      math::Vec3f const &normal,
      float separation,
      float separating_velocity,
      std::array<Dynamic_rigid_body_handle, 2> const &bodies,
      std::array<math::Vec3f, 2> const &relative_positions) noexcept
      : Contact{normal, separation, separating_velocity}, bodies{bodies},
        relative_positions{relative_positions} {}
};

struct Rigid_body_static_body_contact : Contact {
  Dynamic_rigid_body_handle rigid_body;
  Static_rigid_body_handle static_body;
  math::Vec3f relative_position;

  Rigid_body_static_body_contact() = default;

  constexpr Rigid_body_static_body_contact(
      math::Vec3f const &normal,
      float separation,
      float separating_velocity,
      Dynamic_rigid_body_handle dynamic_body,
      Static_rigid_body_handle static_body,
      math::Vec3f const &relative_position) noexcept
      : Contact{normal, separation, separating_velocity},
        rigid_body{dynamic_body}, static_body{static_body},
        relative_position{relative_position} {}
};

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  Particle_particle_contact **particle_contacts;
  Particle_static_body_contact **static_rigid_body_contacts;
  Particle_rigid_body_contact **dynamic_rigid_body_contacts;
  Particle_motion_callback *motion_callback;
  // std::uint64_t collision_flags;
  // std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f position;
  math::Vec3f velocity;
  float inverse_mass;
  float radius;
  Material material;
  std::uint16_t particle_contact_count;
  std::uint16_t static_rigid_body_contact_count;
  std::uint16_t dynamic_rigid_body_contact_count;
  bool marked;
  bool visited;
};

struct Dynamic_rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *broadphase_node;
  Particle_rigid_body_contact **particle_contacts;
  Rigid_body_static_body_contact **static_rigid_body_contacts;
  Rigid_body_rigid_body_contact **dynamic_rigid_body_contacts;
  Dynamic_rigid_body_motion_callback *motion_callback;
  // std::uint64_t collision_flags;
  // std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f position;
  math::Vec3f velocity;
  math::Quatf previous_orientation;
  math::Quatf orientation;
  math::Vec3f angular_velocity;
  float inverse_mass;
  math::Mat3x3f inverse_inertia_tensor;
  Shape shape;
  Material material;
  std::uint16_t particle_contact_count;
  std::uint16_t static_rigid_body_contact_count;
  std::uint16_t dynamic_rigid_body_contact_count;
  bool marked;
  bool visited;
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
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
    }
  }

  Particle_handle alloc() {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for particles"};
    }
    auto const index = _free_indices.back();
    auto const handle = Particle_handle{index};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return handle;
  }

  void free(Particle_handle handle) {
    _free_indices.emplace_back(handle.value);
    _occupancy_bits[handle.value] = false;
  }

  Particle_data *data(Particle_handle handle) {
    return reinterpret_cast<Particle_data *>(
        std::launder(_data.get() + sizeof(Particle_data) * handle.value));
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits[i]) {
        auto const handle = Particle_handle{static_cast<std::uint32_t>(i)};
        f(handle, data(handle));
        ++k;
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::uint32_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

class Rigid_body_storage {
public:
  explicit Rigid_body_storage(std::size_t size)
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Dynamic_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
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
    return reinterpret_cast<Dynamic_rigid_body_data *>(std::launder(
        _data.get() + sizeof(Dynamic_rigid_body_data) * handle.value));
  }

  Dynamic_rigid_body_data const *data(Dynamic_rigid_body_handle handle) const {
    return reinterpret_cast<Dynamic_rigid_body_data const *>(std::launder(
        _data.get() + sizeof(Dynamic_rigid_body_data) * handle.value));
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits[i]) {
        auto const handle =
            Dynamic_rigid_body_handle{static_cast<std::uint32_t>(i)};
        f(handle, data(handle));
        ++k;
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::uint32_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

class Static_rigid_body_storage {
public:
  explicit Static_rigid_body_storage(std::size_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size *
                                            sizeof(Static_rigid_body_data))},
        _free_indices(size), _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
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
    return reinterpret_cast<Static_rigid_body_data *>(std::launder(
        _data.get() + sizeof(Static_rigid_body_data) * handle.value));
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        auto const handle =
            Static_rigid_body_handle{static_cast<std::uint32_t>(i)};
        f(handle, data(handle));
        if (++k == m) {
          return;
        }
      }
    }
  }

private:
  std::unique_ptr<std::byte[]> _data;
  std::vector<std::uint32_t> _free_indices;
  std::vector<bool> _occupancy_bits;
};

class Object_stack {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t capacity) noexcept {
    return Stack_allocator<>::memory_requirement(
        {Array<Object_type>::memory_requirement(capacity),
         Array<Object_handle_t>::memory_requirement(capacity)});
  }

  explicit Object_stack(Block block, std::size_t capacity) noexcept
      : _impl{[&]() {
          auto allocator = Stack_allocator{block};
          auto const object_types_block =
              allocator.alloc(Array<Object_type>::memory_requirement(capacity));
          auto const object_handles_block = allocator.alloc(
              Array<Object_handle_t>::memory_requirement(capacity));
          return Impl{
              .object_types =
                  Array<Object_type>{object_types_block.begin, capacity},
              .object_handles =
                  Array<Object_handle_t>{object_handles_block.begin, capacity}};
        }()} {}

  explicit Object_stack(void *block_begin, std::size_t capacity) noexcept
      : Object_stack{make_block(block_begin, memory_requirement(capacity)),
                     capacity} {}

  bool empty() const noexcept { return _impl.object_types.empty(); }

  std::size_t size() const noexcept { return _impl.object_types.size(); }

  void clear() noexcept {
    _impl.object_types.clear();
    _impl.object_handles.clear();
  }

  void push_back(Particle_handle h) {
    _impl.object_types.push_back(Object_type::particle);
    _impl.object_handles.push_back(h.value);
  }

  void push_back(Dynamic_rigid_body_handle h) {
    _impl.object_types.push_back(Object_type::dynamic_rigid_body);
    _impl.object_handles.push_back(h.value);
  }

  std::variant<Particle_handle, Dynamic_rigid_body_handle> pop_back() noexcept {
    auto const object_type = _impl.object_types.back();
    auto const object_handle_value = _impl.object_handles.back();
    _impl.object_types.pop_back();
    _impl.object_handles.pop_back();
    switch (object_type) {
    case Object_type::particle:
      return Particle_handle{object_handle_value};
    case Object_type::dynamic_rigid_body:
      return Dynamic_rigid_body_handle{object_handle_value};
    case Object_type::static_rigid_body:
    default:
      math::unreachable();
    }
  }

  std::variant<Particle_handle, Dynamic_rigid_body_handle> at(std::size_t i) {
    auto const object_type = _impl.object_types[i];
    auto const object_handle_value = _impl.object_handles[i];
    switch (object_type) {
    case Object_type::particle:
      return Particle_handle{object_handle_value};
    case Object_type::dynamic_rigid_body:
      return Dynamic_rigid_body_handle{object_handle_value};
    case Object_type::static_rigid_body:
    default:
      math::unreachable();
    }
  }

private:
  struct Impl {
    Array<Object_type> object_types;
    Array<Object_handle_t> object_handles;
  };

  Impl _impl;
};

class Contact_stack {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t capacity) noexcept {
    return Stack_allocator<>::memory_requirement(
        {Array<Contact_type>::memory_requirement(capacity),
         Array<Contact *>::memory_requirement(capacity)});
  }

  explicit Contact_stack(Block block, std::size_t const capacity) noexcept
      : _impl{[&]() {
          auto allocator = Stack_allocator{block};
          auto const contact_types_block = allocator.alloc(
              Array<Contact_type>::memory_requirement(capacity));
          auto const contacts_block =
              allocator.alloc(Array<Contact *>::memory_requirement(capacity));
          return Impl{
              .contact_types =
                  Array<Contact_type>{contact_types_block.begin, capacity},
              .contacts = Array<Contact *>{contacts_block.begin, capacity}};
        }()} {}

  explicit Contact_stack(void *block_begin, std::size_t const capacity) noexcept
      : Contact_stack{make_block(block_begin, memory_requirement(capacity)),
                      capacity} {}

  bool empty() const noexcept { return _impl.contact_types.empty(); }

  std::size_t size() const noexcept { return _impl.contact_types.size(); }

  Contact_type const *contact_type_data() const noexcept {
    return _impl.contact_types.data();
  }

  Contact *const *contact_data() const noexcept {
    return _impl.contacts.data();
  }

  std::variant<Particle_particle_contact *,
               Particle_rigid_body_contact *,
               Particle_static_body_contact *,
               Rigid_body_rigid_body_contact *,
               Rigid_body_static_body_contact *>
  at(std::size_t pos) const noexcept {
    auto const result = _impl.contacts[pos];
    switch (_impl.contact_types[pos]) {
    case Contact_type::particle_particle:
      return static_cast<Particle_particle_contact *>(result);
    case Contact_type::particle_dynamic_rigid_body:
      return static_cast<Particle_rigid_body_contact *>(result);
    case Contact_type::particle_static_rigid_body:
      return static_cast<Particle_static_body_contact *>(result);
    case Contact_type::dynamic_rigid_body_dynamic_rigid_body:
      return static_cast<Rigid_body_rigid_body_contact *>(result);
    case Contact_type::dynamic_rigid_body_static_rigid_body:
      return static_cast<Rigid_body_static_body_contact *>(result);
    }
  }

  std::optional<std::variant<Particle_particle_contact *,
                             Particle_rigid_body_contact *,
                             Particle_static_body_contact *,
                             Rigid_body_rigid_body_contact *,
                             Rigid_body_static_body_contact *>>
  find_contact_of_least_separation(float max_separation) const noexcept {
    auto result = static_cast<Contact *const *>(nullptr);
    auto result_separation = max_separation;
    for (auto const &contact : _impl.contacts) {
      if (contact->separation < result_separation) {
        result = &contact;
        result_separation = contact->separation;
      }
    }
    if (result) {
      auto const i = result - _impl.contacts.data();
      switch (_impl.contact_types[i]) {
      case Contact_type::particle_particle:
        return static_cast<Particle_particle_contact *>(*result);
      case Contact_type::particle_dynamic_rigid_body:
        return static_cast<Particle_rigid_body_contact *>(*result);
      case Contact_type::particle_static_rigid_body:
        return static_cast<Particle_static_body_contact *>(*result);
      case Contact_type::dynamic_rigid_body_dynamic_rigid_body:
        return static_cast<Rigid_body_rigid_body_contact *>(*result);
      case Contact_type::dynamic_rigid_body_static_rigid_body:
        return static_cast<Rigid_body_static_body_contact *>(*result);
      default:
        math::unreachable();
      }
    } else {
      return std::nullopt;
    }
  }

  std::optional<std::variant<Particle_particle_contact *,
                             Particle_rigid_body_contact *,
                             Particle_static_body_contact *,
                             Rigid_body_rigid_body_contact *,
                             Rigid_body_static_body_contact *>>
  find_contact_of_least_separating_velocity(
      float max_separating_velocity) const noexcept {
    auto result = static_cast<Contact *const *>(nullptr);
    auto result_separating_velocity = max_separating_velocity;
    for (auto const &contact : _impl.contacts) {
      if (contact->separating_velocity < result_separating_velocity) {
        result = &contact;
        result_separating_velocity = contact->separating_velocity;
      }
    }
    if (result) {
      auto const i = result - _impl.contacts.data();
      switch (_impl.contact_types[i]) {
      case Contact_type::particle_particle:
        return static_cast<Particle_particle_contact *>(*result);
      case Contact_type::particle_dynamic_rigid_body:
        return static_cast<Particle_rigid_body_contact *>(*result);
      case Contact_type::particle_static_rigid_body:
        return static_cast<Particle_static_body_contact *>(*result);
      case Contact_type::dynamic_rigid_body_dynamic_rigid_body:
        return static_cast<Rigid_body_rigid_body_contact *>(*result);
      case Contact_type::dynamic_rigid_body_static_rigid_body:
        return static_cast<Rigid_body_static_body_contact *>(*result);
      default:
        math::unreachable();
      }
    } else {
      return std::nullopt;
    }
  }

  void clear() noexcept {
    _impl.contact_types.clear();
    _impl.contacts.clear();
  }

  void push_back(Particle_particle_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_particle);
    _impl.contacts.push_back(c);
  }

  void push_back(Particle_rigid_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_dynamic_rigid_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Particle_static_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_static_rigid_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Rigid_body_rigid_body_contact *c) {
    _impl.contact_types.push_back(
        Contact_type::dynamic_rigid_body_dynamic_rigid_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Rigid_body_static_body_contact *c) {
    _impl.contact_types.push_back(
        Contact_type::dynamic_rigid_body_static_rigid_body);
    _impl.contacts.push_back(c);
  }

private:
  struct Impl {
    Array<Contact_type> contact_types;
    Array<Contact *> contacts;
  };

  Impl _impl;
};

auto constexpr max_cached_contacts_per_object_pair = 4;
auto constexpr max_cached_contact_drift = 1.0f / 128.0f;
auto constexpr max_cached_contact_separation = 1.0f / 512.0f;

class Contact_cache {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t max_rigid_body_rigid_body_pairs,
                     std::size_t max_rigid_body_static_body_pairs) {
    return Stack_allocator<>::memory_requirement(
        {decltype(_rigid_body_rigid_body_pairs)::memory_requirement(
             max_rigid_body_rigid_body_pairs),
         decltype(_rigid_body_static_body_pairs)::memory_requirement(
             max_rigid_body_static_body_pairs)});
  }

  explicit Contact_cache(Block block,
                         std::size_t max_rigid_body_rigid_body_pairs,
                         std::size_t max_rigid_body_static_body_pairs)
      : Contact_cache{block.begin,
                      max_rigid_body_rigid_body_pairs,
                      max_rigid_body_static_body_pairs} {}

  explicit Contact_cache(void *block_begin,
                         std::size_t max_rigid_body_rigid_body_pairs,
                         std::size_t max_rigid_body_static_body_pairs) {
    auto allocator = Stack_allocator<>{
        make_block(block_begin,
                   memory_requirement(max_rigid_body_rigid_body_pairs,
                                      max_rigid_body_static_body_pairs))};
    _rigid_body_rigid_body_pairs =
        make_map<std::uint64_t, Rigid_body_rigid_body_pair>(
            allocator, max_rigid_body_rigid_body_pairs)
            .second;
    _rigid_body_static_body_pairs =
        make_map<std::uint64_t, Rigid_body_static_body_pair>(
            allocator, max_rigid_body_static_body_pairs)
            .second;
  }

  void mark(Dynamic_rigid_body_handle rigid_body_1,
            Dynamic_rigid_body_handle rigid_body_2) {
    assert(rigid_body_1.value < rigid_body_2.value);
    auto const key = (static_cast<std::uint64_t>(rigid_body_1.value) << 32) |
                     static_cast<std::uint64_t>(rigid_body_2.value);
    auto const it = _rigid_body_rigid_body_pairs.find(key);
    if (it != _rigid_body_rigid_body_pairs.end()) {
      it->second.set_marked();
    }
  }

  void mark(Dynamic_rigid_body_handle rigid_body,
            Static_rigid_body_handle static_body) {
    auto const key = (static_cast<std::uint64_t>(rigid_body.value) << 32) |
                     static_cast<std::uint64_t>(static_body.value);
    auto const it = _rigid_body_static_body_pairs.find(key);
    if (it != _rigid_body_static_body_pairs.end()) {
      it->second.set_marked();
    }
  }

  void update(Rigid_body_storage &dynamic_bodies, Static_rigid_body_storage &) {
    _update_rigid_body_rigid_body_pairs(dynamic_bodies);
    _update_rigid_body_static_body_pairs(dynamic_bodies);
  }

  void cache(Rigid_body_rigid_body_contact const &contact,
             std::array<Vec3f, 2> const &body_positions,
             std::array<Quatf, 2> const &body_orientations) {
    assert(contact.bodies[0].value < contact.bodies[1].value);
    auto const key =
        (static_cast<std::uint64_t>(contact.bodies[0].value) << 32) |
        (static_cast<std::uint64_t>(contact.bodies[1].value));
    auto const it = _rigid_body_rigid_body_pairs.find(key);
    if (it != _rigid_body_rigid_body_pairs.end()) {
      it->second.cache(contact, body_positions, body_orientations);
    } else {
      _rigid_body_rigid_body_pairs.emplace(
          key,
          Rigid_body_rigid_body_pair{
              contact, body_positions, body_orientations});
    }
  }

  void cache(Rigid_body_static_body_contact const &contact,
             Vec3f const &rigid_body_position,
             Quatf const &rigid_body_orientation) {
    auto const key =
        (static_cast<std::uint64_t>(contact.rigid_body.value) << 32) |
        (static_cast<std::uint64_t>(contact.static_body.value));
    auto const it = _rigid_body_static_body_pairs.find(key);
    if (it != _rigid_body_static_body_pairs.end()) {
      it->second.cache(contact, rigid_body_position, rigid_body_orientation);
    } else {
      _rigid_body_static_body_pairs.emplace(
          key,
          Rigid_body_static_body_pair{
              contact, rigid_body_position, rigid_body_orientation});
    }
  }

  std::span<Rigid_body_rigid_body_contact const>
  contacts(Dynamic_rigid_body_handle rigid_body_1,
           Dynamic_rigid_body_handle rigid_body_2) const {
    assert(rigid_body_1.value < rigid_body_2.value);
    auto const key = (static_cast<std::uint64_t>(rigid_body_1.value) << 32) |
                     static_cast<std::uint64_t>(rigid_body_2.value);
    auto const it = _rigid_body_rigid_body_pairs.find(key);
    if (it != _rigid_body_rigid_body_pairs.end()) {
      return it->second.get_contacts();
    } else {
      return {};
    }
  }

  std::span<Rigid_body_rigid_body_contact>
  contacts(Dynamic_rigid_body_handle rigid_body_1,
           Dynamic_rigid_body_handle rigid_body_2) {
    assert(rigid_body_1.value < rigid_body_2.value);
    auto const key = (static_cast<std::uint64_t>(rigid_body_1.value) << 32) |
                     static_cast<std::uint64_t>(rigid_body_2.value);
    auto const it = _rigid_body_rigid_body_pairs.find(key);
    if (it != _rigid_body_rigid_body_pairs.end()) {
      return it->second.get_contacts();
    } else {
      return {};
    }
  }

  std::span<Rigid_body_static_body_contact const>
  contacts(Dynamic_rigid_body_handle dynamic_body,
           Static_rigid_body_handle static_body) const {
    auto const key = (static_cast<std::uint64_t>(dynamic_body.value) << 32) |
                     static_cast<std::uint64_t>(static_body.value);
    auto const it = _rigid_body_static_body_pairs.find(key);
    if (it != _rigid_body_static_body_pairs.end()) {
      return it->second.get_contacts();
    } else {
      return {};
    }
  }

  std::span<Rigid_body_static_body_contact>
  contacts(Dynamic_rigid_body_handle dynamic_body,
           Static_rigid_body_handle static_body) {
    auto const key = (static_cast<std::uint64_t>(dynamic_body.value) << 32) |
                     static_cast<std::uint64_t>(static_body.value);
    auto const it = _rigid_body_static_body_pairs.find(key);
    if (it != _rigid_body_static_body_pairs.end()) {
      return it->second.get_contacts();
    } else {
      return {};
    }
  }

private:
  class Rigid_body_rigid_body_pair {
  public:
    Rigid_body_rigid_body_pair(
        Rigid_body_rigid_body_contact const &contact,
        std::array<math::Vec3f, 2> const &body_positions,
        std::array<math::Quatf, 2> const &body_orientations)
        : _contacts{contact},
          _contact_displacements{
              body_positions[0] + contact.relative_positions[0] -
              body_positions[1] - contact.relative_positions[1]},
          _contact_body_orientations{body_orientations}, _size{1},
          _marked{false}, _fresh{true} {}

    std::span<Rigid_body_rigid_body_contact const>
    get_contacts() const noexcept {
      return {_contacts.data(), _size};
    }

    std::span<Rigid_body_rigid_body_contact> get_contacts() noexcept {
      return {_contacts.data(), _size};
    }

    bool is_marked() const noexcept { return _marked; }

    void set_marked(bool marked = true) noexcept { _marked = marked; }

    void update(Rigid_body_storage const &rigid_bodies) noexcept {
      if (_fresh && _size != 0) {
        _fresh = false;
        _size = 0;
      } else {
        if (_size != 0) {
          auto const handles = _contacts[0].bodies;
          auto const datas = std::array<Dynamic_rigid_body_data const *, 2>{
              rigid_bodies.data(handles[0]), rigid_bodies.data(handles[1])};
          auto const delta_orientation_quaternions = std::array<math::Quatf, 2>{
              normalize(datas[0]->orientation *
                        conjugate(datas[0]->previous_orientation)),
              normalize(datas[1]->orientation *
                        conjugate(datas[1]->previous_orientation))};
          auto const delta_orientations = std::array<math::Mat3x3f, 2>{
              math::Mat3x3f::rotation(delta_orientation_quaternions[0]),
              math::Mat3x3f::rotation(delta_orientation_quaternions[1])};
          for (std::uint8_t i = 0; i < _size;) {
            auto &contact = _contacts[i];
            auto keep_contact = false;
            auto const relative_positions = std::array<Vec3f, 2>{
                delta_orientations[0] * contact.relative_positions[0],
                delta_orientations[1] * contact.relative_positions[1]};
            auto const absolute_positions = std::array<Vec3f, 2>{
                datas[0]->position + relative_positions[0],
                datas[1]->position + relative_positions[1]};
            auto const displacement =
                absolute_positions[0] - absolute_positions[1];
            auto const drift = displacement - _contact_displacements[i];
            auto const drift_squared = math::length_squared(drift);
            if (drift_squared <=
                    max_cached_contact_drift * max_cached_contact_drift &&
                std::abs(normalize(datas[0]->orientation *
                                   conjugate(_contact_body_orientations[i][0]))
                             .w) > 0.999f &&
                std::abs(normalize(datas[1]->orientation *
                                   conjugate(_contact_body_orientations[i][1]))
                             .w) > 0.999f) {
              contact.separation += dot(contact.normal, drift);
              if (contact.separation <= max_cached_contact_separation) {
                contact.separating_velocity = dot(
                    contact.normal,
                    (datas[0]->velocity +
                     cross(datas[0]->angular_velocity, relative_positions[0])) -
                        (datas[1]->velocity + cross(datas[1]->angular_velocity,
                                                    relative_positions[1])));
                contact.relative_positions = relative_positions;
                keep_contact = true;
              }
            }
            if (keep_contact) {
              ++i;
            } else {
              std::shift_left(
                  &_contacts.data()[i], &_contacts.data()[_size], 1);
              std::shift_left(&_contact_displacements.data()[i],
                              &_contact_displacements.data()[_size],
                              1);
              std::shift_left(&_contact_body_orientations.data()[i],
                              &_contact_body_orientations.data()[_size],
                              1);
              --_size;
            }
          }
        }
        if (_size == 0) {
          _fresh = true;
        }
      }
    }

    void cache(Rigid_body_rigid_body_contact const &contact,
               std::array<math::Vec3f, 2> const &body_positions,
               std::array<math::Quatf, 2> const &body_orientations) {
      for (std::uint8_t i = 0; i < _size; ++i) {
        if (math::length_squared(_contacts[i].relative_positions[0] -
                                 contact.relative_positions[0]) <=
                max_cached_contact_drift * max_cached_contact_drift ||
            math::length_squared(_contacts[i].relative_positions[1] -
                                 contact.relative_positions[1]) <=
                max_cached_contact_drift * max_cached_contact_drift) {
          _contacts[i] = contact;
          _contact_displacements[i] =
              (body_positions[0] + contact.relative_positions[0]) -
              (body_positions[1] + contact.relative_positions[1]);
          _contact_body_orientations[i] = body_orientations;
          return;
        }
      }
      if (_size < max_cached_contacts_per_object_pair) {
        _contacts[_size] = contact;
        _contact_displacements[_size] =
            (body_positions[0] + contact.relative_positions[0]) -
            (body_positions[1] + contact.relative_positions[1]);
        _contact_body_orientations[_size] = body_orientations;
        ++_size;
      } else {
        auto const min_it = std::min_element(
            _contacts.begin(),
            _contacts.end(),
            [&](auto const &lhs, auto const &rhs) {
              return std::min(length_squared(lhs.relative_positions[0] -
                                             contact.relative_positions[0]),
                              length_squared(lhs.relative_positions[1] -
                                             contact.relative_positions[1])) <
                     std::min(length_squared(rhs.relative_positions[0] -
                                             contact.relative_positions[0]),
                              length_squared(rhs.relative_positions[1] -
                                             contact.relative_positions[1]));
            });
        auto const i = min_it - _contacts.begin();
        _contacts[i] = contact;
        _contact_displacements[i] =
            (body_positions[0] + contact.relative_positions[0]) -
            (body_positions[1] + contact.relative_positions[1]);
        _contact_body_orientations[i] = body_orientations;
      }
    }

  private:
    std::array<Rigid_body_rigid_body_contact,
               max_cached_contacts_per_object_pair>
        _contacts;
    std::array<Vec3f, max_cached_contacts_per_object_pair>
        _contact_displacements;
    std::array<std::array<Quatf, 2>, max_cached_contacts_per_object_pair>
        _contact_body_orientations;
    std::uint8_t _size;
    bool _marked;
    bool _fresh;
  };

  class Rigid_body_static_body_pair {
  public:
    Rigid_body_static_body_pair(Rigid_body_static_body_contact const &contact,
                                Vec3f const &rigid_body_position,
                                Quatf const &rigid_body_orientation)
        : _contacts{contact},
          _contact_positions{rigid_body_position + contact.relative_position},
          _contact_body_orientations{rigid_body_orientation}, _size{1},
          _marked{false}, _fresh{true} {}

    std::span<Rigid_body_static_body_contact const>
    get_contacts() const noexcept {
      return {_contacts.data(), _size};
    }

    std::span<Rigid_body_static_body_contact> get_contacts() noexcept {
      return {_contacts.data(), _size};
    }

    bool is_marked() const noexcept { return _marked; }

    void set_marked(bool marked = true) noexcept { _marked = marked; }

    void update(Rigid_body_storage const &rigid_bodies) noexcept {
      if (_fresh && _size != 0) {
        _fresh = false;
        _size = 0;
      } else {
        if (_size != 0) {
          auto const rigid_body_handle = _contacts[0].rigid_body;
          auto const rigid_body_data = rigid_bodies.data(rigid_body_handle);
          auto const delta_orientation_quaternion =
              normalize(rigid_body_data->orientation *
                        conjugate(rigid_body_data->previous_orientation));
          auto const delta_orientation_matrix =
              Mat3x3f::rotation(delta_orientation_quaternion);
          for (std::uint8_t i = 0; i < _size;) {
            auto &contact = _contacts[i];
            auto keep_contact = false;
            auto const relative_position =
                delta_orientation_matrix * contact.relative_position;
            auto const absolute_position =
                rigid_body_data->position + relative_position;
            auto const drift = absolute_position - _contact_positions[i];
            auto const drift_squared = length_squared(drift);
            if (drift_squared <=
                    max_cached_contact_drift * max_cached_contact_drift &&
                std::abs(normalize(rigid_body_data->orientation *
                                   conjugate(_contact_body_orientations[i]))
                             .w) > 0.999f) {
              contact.separation += dot(contact.normal, drift);
              if (contact.separation <= max_cached_contact_separation) {
                contact.separating_velocity =
                    dot(contact.normal,
                        rigid_body_data->velocity +
                            cross(rigid_body_data->angular_velocity,
                                  relative_position));
                contact.relative_position = relative_position;
                keep_contact = true;
              }
            }
            if (keep_contact) {
              ++i;
            } else {
              std::shift_left(
                  &_contacts.data()[i], &_contacts.data()[_size], 1);
              std::shift_left(&_contact_positions.data()[i],
                              &_contact_positions.data()[_size],
                              1);
              std::shift_left(&_contact_body_orientations.data()[i],
                              &_contact_body_orientations.data()[_size],
                              1);
              --_size;
            }
          }
        }
        if (_size == 0) {
          _fresh = true;
        }
      }
    }

    void cache(Rigid_body_static_body_contact const &contact,
               Vec3f const &rigid_body_position,
               Quatf const &rigid_body_orientation) {
      for (std::uint8_t i = 0; i < _size; ++i) {
        if (length_squared(_contacts[i].relative_position -
                           contact.relative_position) <=
            max_cached_contact_drift * max_cached_contact_drift) {
          _contacts[i] = contact;
          _contact_positions[i] =
              rigid_body_position + contact.relative_position;
          _contact_body_orientations[i] = rigid_body_orientation;
          return;
        }
      }
      if (_size < max_cached_contacts_per_object_pair) {
        _contacts[_size++] = contact;
      } else {
        auto const min_it = std::min_element(
            _contacts.begin(),
            _contacts.end(),
            [&](auto const &lhs, auto const &rhs) {
              return length_squared(lhs.relative_position -
                                    contact.relative_position) <
                     length_squared(rhs.relative_position -
                                    contact.relative_position);
            });
        auto const i = min_it - _contacts.begin();
        _contacts[i] = contact;
        _contact_positions[i] = rigid_body_position + contact.relative_position;
        _contact_body_orientations[i] = rigid_body_orientation;
      }
    }

  private:
    std::array<Rigid_body_static_body_contact,
               max_cached_contacts_per_object_pair>
        _contacts;
    std::array<Vec3f, max_cached_contacts_per_object_pair> _contact_positions;
    std::array<Quatf, max_cached_contacts_per_object_pair>
        _contact_body_orientations;
    std::uint8_t _size;
    bool _marked;
    bool _fresh;
  };

  void _update_rigid_body_rigid_body_pairs(Rigid_body_storage &rigid_bodies) {
    auto it = _rigid_body_rigid_body_pairs.begin();
    while (it != _rigid_body_rigid_body_pairs.end()) {
      auto &node = it->second;
      if (node.is_marked()) {
        node.set_marked(false);
        node.update(rigid_bodies);
        ++it;
      } else {
        it = _rigid_body_rigid_body_pairs.erase(it);
      }
    }
  }

  void _update_rigid_body_static_body_pairs(Rigid_body_storage &rigid_bodies) {
    auto it = _rigid_body_static_body_pairs.begin();
    while (it != _rigid_body_static_body_pairs.end()) {
      auto &node = it->second;
      if (node.is_marked()) {
        node.set_marked(false);
        node.update(rigid_bodies);
        ++it;
      } else {
        it = _rigid_body_static_body_pairs.erase(it);
      }
    }
  }

  util::Map<std::uint64_t, Rigid_body_rigid_body_pair>
      _rigid_body_rigid_body_pairs;
  util::Map<std::uint64_t, Rigid_body_static_body_pair>
      _rigid_body_static_body_pairs;
};

auto const velocity_damping_factor = 0.99f;
auto const max_rotational_displacement_coefficient = 0.2f;
} // namespace

class Space::Impl {
public:
  friend class Space;

  explicit Impl(Space_create_info const &create_info,
                std::unique_ptr<std::byte[]> system_allocation,
                void *aabb_tree_block,
                void *close_particle_particle_pairs_begin,
                void *close_particle_rigid_body_pairs_begin,
                void *close_particle_static_body_pairs_begin,
                void *close_rigid_body_rigid_body_pairs_begin,
                void *close_rigid_body_static_body_pairs_begin,
                void *particle_particle_contacts_begin,
                void *particle_rigid_body_contacts_begin,
                void *particle_static_body_contacts_begin,
                void *rigid_body_rigid_body_contacts_begin,
                void *rigid_body_static_body_contacts_begin,
                void *particle_particle_contact_pointers_begin,
                void *particle_rigid_body_contact_pointers_begin,
                void *particle_static_body_contact_pointers_begin,
                void *rigid_body_rigid_body_contact_pointers_begin,
                void *rigid_body_static_body_contact_pointers_begin,
                void *island_fringe_begin,
                void *island_contacts_begin,
                void *contact_cache_block)
      : _system_allocation{std::move(system_allocation)},
        _aabb_tree{aabb_tree_block,
                   create_info.max_aabb_tree_leaf_nodes,
                   create_info.max_aabb_tree_internal_nodes},
        _particles{create_info.max_particles},
        _static_bodies{create_info.max_static_rigid_bodies},
        _rigid_bodies{create_info.max_dynamic_rigid_bodies},
        _close_particle_particle_pairs{
            close_particle_particle_pairs_begin,
            create_info.max_particle_particle_contacts},
        _close_particle_rigid_body_pairs{
            close_particle_rigid_body_pairs_begin,
            create_info.max_particle_rigid_body_contacts},
        _close_particle_static_body_pairs{
            close_particle_static_body_pairs_begin,
            create_info.max_particle_static_body_contacts},
        _close_rigid_body_rigid_body_pairs{
            close_rigid_body_rigid_body_pairs_begin,
            create_info.max_rigid_body_rigid_body_contacts},
        _close_rigid_body_static_body_pairs{
            close_rigid_body_static_body_pairs_begin,
            create_info.max_rigid_body_static_body_contacts},
        _particle_particle_contacts{particle_particle_contacts_begin,
                                    create_info.max_particle_particle_contacts},
        _particle_rigid_body_contacts{
            particle_rigid_body_contacts_begin,
            create_info.max_particle_rigid_body_contacts},
        _particle_static_body_contacts{
            particle_static_body_contacts_begin,
            create_info.max_particle_static_body_contacts},
        _rigid_body_rigid_body_contacts{
            rigid_body_rigid_body_contacts_begin,
            create_info.max_rigid_body_rigid_body_contacts},
        _rigid_body_static_body_contacts{
            rigid_body_static_body_contacts_begin,
            create_info.max_rigid_body_static_body_contacts},
        _particle_particle_contact_pointers{
            particle_particle_contact_pointers_begin,
            2 * create_info.max_particle_particle_contacts},
        _particle_rigid_body_contact_pointers{
            particle_rigid_body_contact_pointers_begin,
            2 * create_info.max_particle_rigid_body_contacts},
        _particle_static_body_contact_pointers{
            particle_static_body_contact_pointers_begin,
            create_info.max_particle_static_body_contacts},
        _rigid_body_rigid_body_contact_pointers{
            rigid_body_rigid_body_contact_pointers_begin,
            2 * create_info.max_rigid_body_rigid_body_contacts},
        _rigid_body_static_body_contact_pointers{
            rigid_body_static_body_contact_pointers_begin,
            create_info.max_rigid_body_static_body_contacts},
        _island_fringe{island_fringe_begin,
                       create_info.max_island_object_count},
        _island_contacts{island_contacts_begin,
                         create_info.max_island_contact_count},
        _contact_cache{contact_cache_block,
                       create_info.max_rigid_body_rigid_body_contacts,
                       create_info.max_rigid_body_static_body_contacts},
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
                      .previous_position = create_info.position,
                      .position = create_info.position,
                      .velocity = create_info.velocity,
                      .inverse_mass = 1.0f / create_info.mass,
                      .radius = create_info.radius,
                      .material = create_info.material,
                      .marked = false,
                      .visited = false};
    return handle;
  }

  void destroy_particle(Particle_handle handle) {
    _aabb_tree.destroy_leaf(_particles.data(handle)->aabb_tree_node);
    _particles.free(handle);
  }

  Dynamic_rigid_body_handle
  create_dynamic_rigid_body(Dynamic_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const handle = _rigid_bodies.alloc();
    auto const bounds = physics::bounds(create_info.shape, transform);
    new (_rigid_bodies.data(handle)) Dynamic_rigid_body_data{
        .broadphase_node = _aabb_tree.create_leaf(bounds, handle),
        .motion_callback = create_info.motion_callback,
        // .collision_flags = create_info.collision_flags,
        // .collision_mask = create_info.collision_mask,
        .previous_position = create_info.position,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .previous_orientation = create_info.orientation,
        .orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
        .inverse_mass = 1.0f / create_info.mass,
        .inverse_inertia_tensor = inverse(create_info.inertia_tensor),
        .shape = create_info.shape,
        .material = create_info.material,
        .marked = false,
        .visited = false};
    return handle;
  }

  void destroy_dynamic_rigid_body(Dynamic_rigid_body_handle handle) {
    _aabb_tree.destroy_leaf(_rigid_bodies.data(handle)->broadphase_node);
    _rigid_bodies.free(handle);
  }

  Static_rigid_body_handle
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    auto const transform =
        math::Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    // TODO: cleanup allocated handle if create_leaf fails
    auto const handle = _static_bodies.alloc();
    new (_static_bodies.data(handle)) Static_rigid_body_data{
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
    _aabb_tree.destroy_leaf(_static_bodies.data(handle)->aabb_tree_node);
    _static_bodies.free(handle);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    build_aabb_tree(simulate_info.delta_time);
    find_potential_contacts();
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate(h);
      find_contacts();
      solve(h,
            simulate_info.min_position_iterations_per_contact,
            simulate_info.max_position_iterations_per_contact,
            simulate_info.min_velocity_iterations_per_contact,
            simulate_info.max_velocity_iterations_per_contact,
            simulate_info.early_out_contact_separation,
            simulate_info.early_out_contact_separating_velocity);
    }
    call_particle_motion_callbacks();
    call_dynamic_rigid_body_motion_callbacks();
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const constant_safety_term = 0.1f;
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
    _rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->broadphase_node->bounds = expand(
              bounds(data->shape,
                     math::Mat3x4f::rigid(data->position, data->orientation)),
              constant_safety_term +
                  velocity_safety_factor * math::length(data->velocity) *
                      delta_time +
                  gravity_safety_term);
        });
    _aabb_tree.build();
  }

  void find_potential_contacts() {
    _close_particle_particle_pairs.clear();
    _close_particle_static_body_pairs.clear();
    _close_particle_rigid_body_pairs.clear();
    _close_rigid_body_static_body_pairs.clear();
    _close_rigid_body_rigid_body_pairs.clear();
    _aabb_tree.for_each_overlapping_leaf_pair(
        [this](Aabb_tree_payload_t const &first_payload,
               Aabb_tree_payload_t const &second_payload) {
          std::visit(
              [this, &second_payload](auto &&first_handle) {
                using T = std::decay_t<decltype(first_handle)>;
                if constexpr (std::is_same_v<T, Particle_handle>) {
                  std::visit(
                      [this, first_handle](auto &&second_handle) {
                        using U = std::decay_t<decltype(second_handle)>;
                        if constexpr (std::is_same_v<U, Particle_handle>) {
                          _close_particle_particle_pairs.push_back(
                              {first_handle, second_handle});
                        } else if constexpr (std::is_same_v<
                                                 U,
                                                 Static_rigid_body_handle>) {
                          _close_particle_static_body_pairs.push_back(
                              {first_handle, second_handle});
                        } else {
                          static_assert(
                              std::is_same_v<U, Dynamic_rigid_body_handle>);
                          _close_particle_rigid_body_pairs.push_back(
                              {first_handle, second_handle});
                        }
                      },
                      second_payload);
                } else if constexpr (std::is_same_v<T,
                                                    Static_rigid_body_handle>) {
                  std::visit(
                      [this, first_handle](auto &&second_handle) {
                        using U = std::decay_t<decltype(second_handle)>;
                        if constexpr (std::is_same_v<U, Particle_handle>) {
                          _close_particle_static_body_pairs.push_back(
                              {second_handle, first_handle});
                        } else if constexpr (std::is_same_v<
                                                 U,
                                                 Dynamic_rigid_body_handle>) {
                          _close_rigid_body_static_body_pairs.push_back(
                              {second_handle, first_handle});
                        }
                      },
                      second_payload);
                } else {
                  static_assert(std::is_same_v<T, Dynamic_rigid_body_handle>);
                  std::visit(
                      [this, first_handle](auto &&second_handle) {
                        using U = std::decay_t<decltype(second_handle)>;
                        if constexpr (std::is_same_v<U, Particle_handle>) {
                          _close_particle_rigid_body_pairs.push_back(
                              {second_handle, first_handle});
                        } else if constexpr (std::is_same_v<
                                                 U,
                                                 Static_rigid_body_handle>) {
                          _close_rigid_body_static_body_pairs.push_back(
                              {first_handle, second_handle});
                        } else if constexpr (std::is_same_v<
                                                 U,
                                                 Dynamic_rigid_body_handle>) {
                          auto handles = std::pair{first_handle, second_handle};
                          if (handles.first.value > handles.second.value) {
                            std::swap(handles.first, handles.second);
                          }
                          _close_rigid_body_rigid_body_pairs.push_back(handles);
                        }
                      },
                      second_payload);
                }
              },
              first_payload);
        });
  }

  void integrate(float h) {
    auto const time_compensated_damping_factor =
        std::pow(velocity_damping_factor, h);
    integrate_particles(h, time_compensated_damping_factor);
    integrate_rigid_bodies(h, time_compensated_damping_factor);
  }

  void integrate_particles(float h, float time_compensated_damping_factor) {
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      data->previous_position = data->position;
      data->velocity += h * _gravitational_acceleration;
      data->velocity *= time_compensated_damping_factor;
      data->position += h * data->velocity;
    });
  }

  void integrate_rigid_bodies(float h, float time_compensated_damping_factor) {
    _rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->previous_position = data->position;
          data->velocity += h * _gravitational_acceleration;
          data->velocity *= time_compensated_damping_factor;
          data->position += h * data->velocity;
          data->previous_orientation = data->orientation;
          data->angular_velocity *= time_compensated_damping_factor;
          data->orientation +=
              math::Quatf{0.0f, 0.5f * h * data->angular_velocity} *
              data->orientation;
          data->orientation = math::normalize(data->orientation);
        });
  }

  void find_contacts() {
    reset_contact_counts();
    find_new_contacts();
    _contact_cache.update(_rigid_bodies, _static_bodies);
    cache_new_contacts();
    count_contacts();
    reset_contact_pointer_storage();
    allocate_particle_contact_pointers();
    allocate_rigid_body_contact_pointers();
    reset_contact_counts();
    assign_and_count_particle_particle_contact_pointers();
    assign_and_count_particle_rigid_body_contact_pointers();
    assign_and_count_particle_static_body_contact_pointers();
    assign_and_count_rigid_body_rigid_body_contact_pointers();
    assign_and_count_rigid_body_static_body_contact_pointers();
  }

  void reset_contact_counts() {
    reset_particle_contact_counts();
    reset_dynamic_rigid_body_contact_counts();
  }

  void reset_particle_contact_counts() {
    _particles.for_each([](Particle_handle, Particle_data *data) {
      data->particle_contact_count = 0;
      data->dynamic_rigid_body_contact_count = 0;
      data->static_rigid_body_contact_count = 0;
    });
  }

  void reset_dynamic_rigid_body_contact_counts() {
    _rigid_bodies.for_each(
        [](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->particle_contact_count = 0;
          data->dynamic_rigid_body_contact_count = 0;
          data->static_rigid_body_contact_count = 0;
        });
  }

  void find_new_contacts() {
    find_new_particle_particle_contacts();
    find_new_particle_rigid_body_contacts();
    find_new_particle_static_body_contacts();
    find_new_rigid_body_rigid_body_contacts();
    find_new_rigid_body_static_body_contacts();
  }

  void find_new_particle_particle_contacts() {
    _particle_particle_contacts.clear();
    for (auto const &particle_handles : _close_particle_particle_pairs) {
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
            {normal,
             separation,
             separating_velocity,
             {particle_handles.first, particle_handles.second}});
        for (auto i = 0; i != 2; ++i) {
          ++particle_datas[i]->particle_contact_count;
        }
      }
    }
  }

  void find_new_particle_rigid_body_contacts() {
    _particle_rigid_body_contacts.clear();
    for (auto [particle_handle, body_handle] :
         _close_particle_rigid_body_pairs) {
      auto const particle_data = _particles.data(particle_handle);
      auto const body_data = _rigid_bodies.data(body_handle);
      auto const body_transform =
          math::Mat3x4f::rigid(body_data->position, body_data->orientation);
      auto const inverse_body_transform = math::rigid_inverse(body_transform);
      if (auto const contact_geometry =
              particle_shape_positionful_contact_geometry(
                  particle_data->position,
                  particle_data->radius,
                  body_data->shape,
                  body_transform,
                  inverse_body_transform)) {
        auto const body_relative_position =
            contact_geometry->position - body_data->position;
        auto const relative_velocity =
            particle_data->velocity -
            (body_data->velocity +
             math::cross(body_data->angular_velocity, body_relative_position));
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        _particle_rigid_body_contacts.push_back({contact_geometry->normal,
                                                 contact_geometry->separation,
                                                 separating_velocity,
                                                 particle_handle,
                                                 body_handle,
                                                 body_relative_position});
        ++particle_data->dynamic_rigid_body_contact_count;
        ++body_data->particle_contact_count;
      }
    }
  }

  void find_new_particle_static_body_contacts() {
    _particle_static_body_contacts.clear();
    for (auto [particle_handle, body_handle] :
         _close_particle_static_body_pairs) {
      auto const particle_data = _particles.data(particle_handle);
      auto const body_data = _static_bodies.data(body_handle);
      if (auto const contact_geometry =
              particle_shape_positionless_contact_geometry(
                  particle_data->position,
                  particle_data->radius,
                  body_data->shape,
                  body_data->transform,
                  body_data->inverse_transform)) {
        auto const separating_velocity =
            math::dot(particle_data->velocity, contact_geometry->normal);
        _particle_static_body_contacts.push_back({contact_geometry->normal,
                                                  contact_geometry->separation,
                                                  separating_velocity,
                                                  particle_handle,
                                                  body_handle});
        ++particle_data->static_rigid_body_contact_count;
      }
    }
  }

  void find_new_rigid_body_rigid_body_contacts() {
    _rigid_body_rigid_body_contacts.clear();
    for (auto const &body_handles : _close_rigid_body_rigid_body_pairs) {
      auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
          _rigid_bodies.data(body_handles.first),
          _rigid_bodies.data(body_handles.second)};
      auto const body_transforms = std::array<math::Mat3x4f, 2>{
          math::Mat3x4f::rigid(body_datas[0]->position,
                               body_datas[0]->orientation),
          math::Mat3x4f::rigid(body_datas[1]->position,
                               body_datas[1]->orientation)};
      auto const body_inverse_transforms =
          std::array<math::Mat3x4f, 2>{math::rigid_inverse(body_transforms[0]),
                                       math::rigid_inverse(body_transforms[1])};
      if (auto const contact_geometry =
              shape_shape_contact_geometry(body_datas[0]->shape,
                                           body_transforms[0],
                                           body_inverse_transforms[0],
                                           body_datas[1]->shape,
                                           body_transforms[1],
                                           body_inverse_transforms[1])) {
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
        _rigid_body_rigid_body_contacts.push_back(
            {contact_geometry->normal,
             contact_geometry->separation,
             separating_velocity,
             {body_handles.first, body_handles.second},
             body_relative_contact_positions});
        _contact_cache.mark(body_handles.first, body_handles.second);
        // for (auto i = 0; i != 2; ++i) {
        //   ++body_datas[i]->dynamic_rigid_body_contact_count;
        // }
      }
    }
  }

  void find_new_rigid_body_static_body_contacts() {
    _rigid_body_static_body_contacts.clear();
    for (auto [rigid_body_handle, static_body_handle] :
         _close_rigid_body_static_body_pairs) {
      auto const rigid_body_data = _rigid_bodies.data(rigid_body_handle);
      auto const rigid_body_transform = math::Mat3x4f::rigid(
          rigid_body_data->position, rigid_body_data->orientation);
      auto const rigid_body_inverse_transform =
          math::rigid_inverse(rigid_body_transform);
      auto const static_body_data = _static_bodies.data(static_body_handle);
      if (auto const contact_geometry = shape_shape_contact_geometry(
              rigid_body_data->shape,
              rigid_body_transform,
              rigid_body_inverse_transform,
              static_body_data->shape,
              static_body_data->transform,
              static_body_data->inverse_transform)) {
        auto const rigid_body_relative_contact_position =
            contact_geometry->position - rigid_body_data->position;
        auto const relative_velocity =
            rigid_body_data->velocity +
            math::cross(rigid_body_data->angular_velocity,
                        rigid_body_relative_contact_position);
        auto const separating_velocity =
            math::dot(relative_velocity, contact_geometry->normal);
        // ++dynamic_body_data->static_rigid_body_contact_count;
        _rigid_body_static_body_contacts.push_back(
            {contact_geometry->normal,
             contact_geometry->separation,
             separating_velocity,
             rigid_body_handle,
             static_body_handle,
             rigid_body_relative_contact_position});
        _contact_cache.mark(rigid_body_handle, static_body_handle);
        // if (contact_geometry->separation > -0.01f) {
        //   std::cout << "cacheable contact\n";
        // }
      }
    }
  }

  void cache_new_contacts() {
    for (auto const &contact : _rigid_body_rigid_body_contacts) {
      auto const data = std::array<Dynamic_rigid_body_data *, 2>{
          _rigid_bodies.data(contact.bodies[0]),
          _rigid_bodies.data(contact.bodies[1])};
      _contact_cache.cache(contact,
                           {data[0]->position, data[1]->position},
                           {data[0]->orientation, data[1]->orientation});
    }
    for (auto const &contact : _rigid_body_static_body_contacts) {
      auto const data = _rigid_bodies.data(contact.rigid_body);
      _contact_cache.cache(contact, data->position, data->orientation);
    }
  }

  void count_contacts() {
    for (auto const &new_contact : _rigid_body_rigid_body_contacts) {
      auto const datas = std::array<Dynamic_rigid_body_data *, 2>{
          _rigid_bodies.data(new_contact.bodies[0]),
          _rigid_bodies.data(new_contact.bodies[1])};
      auto const cached_contacts =
          _contact_cache.contacts(new_contact.bodies[0], new_contact.bodies[1]);
      datas[0]->dynamic_rigid_body_contact_count += cached_contacts.size();
      datas[1]->dynamic_rigid_body_contact_count += cached_contacts.size();
    }
    for (auto const &contact : _rigid_body_static_body_contacts) {
      auto const rigid_body_data = _rigid_bodies.data(contact.rigid_body);
      rigid_body_data->static_rigid_body_contact_count +=
          _contact_cache.contacts(contact.rigid_body, contact.static_body)
              .size();
    }
  }

  void reset_contact_pointer_storage() {
    _particle_particle_contact_pointers.clear();
    _particle_rigid_body_contact_pointers.clear();
    _particle_static_body_contact_pointers.clear();
    _rigid_body_rigid_body_contact_pointers.clear();
    _rigid_body_static_body_contact_pointers.clear();
  }

  void allocate_particle_contact_pointers() {
    _particles.for_each([this](Particle_handle, Particle_data *data) {
      data->particle_contacts = _particle_particle_contact_pointers.end();
      data->dynamic_rigid_body_contacts =
          _particle_rigid_body_contact_pointers.end();
      data->static_rigid_body_contacts =
          _particle_static_body_contact_pointers.end();
      _particle_particle_contact_pointers.resize(
          _particle_particle_contact_pointers.size() +
          data->particle_contact_count);
      _particle_rigid_body_contact_pointers.resize(
          _particle_rigid_body_contact_pointers.size() +
          data->dynamic_rigid_body_contact_count);
      _particle_static_body_contact_pointers.resize(
          _particle_static_body_contact_pointers.size() +
          data->static_rigid_body_contact_count);
    });
  }

  void allocate_rigid_body_contact_pointers() {
    _rigid_bodies.for_each(
        [this](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->particle_contacts = _particle_rigid_body_contact_pointers.end();
          data->dynamic_rigid_body_contacts =
              _rigid_body_rigid_body_contact_pointers.end();
          data->static_rigid_body_contacts =
              _rigid_body_static_body_contact_pointers.end();
          _particle_rigid_body_contact_pointers.resize(
              _particle_rigid_body_contact_pointers.size() +
              data->particle_contact_count);
          _rigid_body_rigid_body_contact_pointers.resize(
              _rigid_body_rigid_body_contact_pointers.size() +
              data->dynamic_rigid_body_contact_count);
          _rigid_body_static_body_contact_pointers.resize(
              _rigid_body_static_body_contact_pointers.size() +
              data->static_rigid_body_contact_count);
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

  void assign_and_count_particle_rigid_body_contact_pointers() {
    for (auto &contact : _particle_rigid_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      auto const dynamic_rigid_body_data = _rigid_bodies.data(contact.body);
      particle_data->dynamic_rigid_body_contacts
          [particle_data->dynamic_rigid_body_contact_count++] = &contact;
      dynamic_rigid_body_data->particle_contacts
          [dynamic_rigid_body_data->particle_contact_count++] = &contact;
    }
  }

  void assign_and_count_particle_static_body_contact_pointers() {
    for (auto &contact : _particle_static_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      particle_data->static_rigid_body_contacts
          [particle_data->static_rigid_body_contact_count++] = &contact;
    }
  }

  void assign_and_count_rigid_body_rigid_body_contact_pointers() {
    for (auto &new_contact : _rigid_body_rigid_body_contacts) {
      auto const datas = std::array<Dynamic_rigid_body_data *, 2>{
          _rigid_bodies.data(new_contact.bodies[0]),
          _rigid_bodies.data(new_contact.bodies[1])};
      auto const cached_contacts =
          _contact_cache.contacts(new_contact.bodies[0], new_contact.bodies[1]);
      for (auto &cached_contact : cached_contacts) {
        for (auto i = 0; i != 2; ++i) {
          datas[i]->dynamic_rigid_body_contacts
              [datas[i]->dynamic_rigid_body_contact_count++] = &cached_contact;
        }
      }
    }
  }

  void assign_and_count_rigid_body_static_body_contact_pointers() {
    for (auto &new_contact : _rigid_body_static_body_contacts) {
      auto const rigid_body_data = _rigid_bodies.data(new_contact.rigid_body);
      auto const cached_contacts = _contact_cache.contacts(
          new_contact.rigid_body, new_contact.static_body);
      for (auto &cached_contact : cached_contacts) {
        rigid_body_data->static_rigid_body_contacts
            [rigid_body_data->static_rigid_body_contact_count++] =
            &cached_contact;
      }
    }
  }

  void solve(float h,
             int min_position_iterations_per_contact,
             int max_position_iterations_per_contact,
             int min_velocity_iterations_per_contact,
             int max_velocity_iterations_per_contact,
             float early_out_contact_separation,
             float early_out_contact_separating_velocity) {
    auto const gravitational_velocity_delta = _gravitational_acceleration * h;
    auto const max_separating_velocity_for_bounce =
        -2.0f * math::length(gravitational_velocity_delta);
    unmark_particles();
    unmark_dynamic_rigid_bodies();
    auto const island_object_visitor = [this](auto &&handle) {
      using T = std::decay_t<decltype(handle)>;
      if constexpr (std::is_same_v<T, Particle_handle>) {
        // std::cout << "visiting particle" << std::endl;
        auto const data = _particles.data(handle);
        data->visited = true;
        for (auto const contact :
             std::span{data->particle_contacts, data->particle_contact_count}) {
          auto const other_handle =
              contact->particles[contact->particles[0] == handle ? 1 : 0];
          auto const other_data = _particles.data(other_handle);
          if (!other_data->marked) {
            other_data->marked = true;
            _island_fringe.push_back(other_handle);
          }
          if (!other_data->visited) {
            _island_contacts.push_back(contact);
          }
        }
        for (auto const contact :
             std::span{data->dynamic_rigid_body_contacts,
                       data->dynamic_rigid_body_contact_count}) {
          auto const other_handle = contact->body;
          auto const other_data = _rigid_bodies.data(other_handle);
          if (!other_data->marked) {
            other_data->marked = true;
            _island_fringe.push_back(other_handle);
          }
          if (!other_data->visited) {
            _island_contacts.push_back(contact);
          }
        }
        for (auto const contact :
             std::span{data->static_rigid_body_contacts,
                       data->static_rigid_body_contact_count}) {
          _island_contacts.push_back(contact);
        }
      } else if constexpr (std::is_same_v<T, Dynamic_rigid_body_handle>) {
        // std::cout << "visiting dynamic rigid body" << std::endl;
        auto const data = _rigid_bodies.data(handle);
        data->visited = true;
        for (auto const contact :
             std::span{data->particle_contacts, data->particle_contact_count}) {
          auto const other_handle = contact->particle;
          auto const other_data = _particles.data(other_handle);
          if (!other_data->marked) {
            other_data->marked = true;
            _island_fringe.push_back(other_handle);
          }
          if (!other_data->visited) {
            _island_contacts.push_back(contact);
          }
        }
        for (auto const contact :
             std::span{data->dynamic_rigid_body_contacts,
                       data->dynamic_rigid_body_contact_count}) {
          auto const other_handle =
              contact->bodies[contact->bodies[0] == handle ? 1 : 0];
          auto const other_data = _rigid_bodies.data(other_handle);
          if (!other_data->marked) {
            other_data->marked = true;
            _island_fringe.push_back(other_handle);
          }
          if (!other_data->visited) {
            _island_contacts.push_back(contact);
          }
        }
        for (auto const contact :
             std::span{data->static_rigid_body_contacts,
                       data->static_rigid_body_contact_count}) {
          _island_contacts.push_back(contact);
        }
      }
    };
    _particles.for_each([&, this](Particle_handle handle, Particle_data *data) {
      if (!data->marked) {
        data->marked = true;
        initialize_island(handle);
        do {
          std::visit(island_object_visitor, _island_fringe.pop_back());
        } while (!_island_fringe.empty());
        solve_current_island(min_position_iterations_per_contact,
                             max_position_iterations_per_contact,
                             min_velocity_iterations_per_contact,
                             max_velocity_iterations_per_contact,
                             early_out_contact_separation,
                             early_out_contact_separating_velocity,
                             gravitational_velocity_delta,
                             max_separating_velocity_for_bounce);
      }
    });
    _rigid_bodies.for_each([&, this](Dynamic_rigid_body_handle handle,
                                     Dynamic_rigid_body_data *data) {
      if (!data->marked) {
        data->marked = true;
        initialize_island(handle);
        do {
          std::visit(island_object_visitor, _island_fringe.pop_back());
        } while (!_island_fringe.empty());
        solve_current_island(min_position_iterations_per_contact,
                             max_position_iterations_per_contact,
                             min_position_iterations_per_contact,
                             max_velocity_iterations_per_contact,
                             early_out_contact_separation,
                             early_out_contact_separating_velocity,
                             gravitational_velocity_delta,
                             max_separating_velocity_for_bounce);
      }
    });
  }

  void unmark_particles() {
    _particles.for_each([](Particle_handle, Particle_data *data) {
      data->marked = false;
      data->visited = false;
    });
  }

  void unmark_dynamic_rigid_bodies() {
    _rigid_bodies.for_each(
        [](Dynamic_rigid_body_handle, Dynamic_rigid_body_data *data) {
          data->marked = false;
          data->visited = false;
        });
  }

  void initialize_island(Particle_handle handle) {
    assert(_island_fringe.empty());
    _island_fringe.push_back(handle);
    _island_contacts.clear();
  }

  void initialize_island(Dynamic_rigid_body_handle handle) {
    assert(_island_fringe.empty());
    _island_fringe.push_back(handle);
    _island_contacts.clear();
  }

  void solve_current_island(int min_position_iterations_per_contact,
                            int max_position_iterations_per_contact,
                            int min_velocity_iterations_per_contact,
                            int max_velocity_iterations_per_contact,
                            float early_out_contact_separation,
                            float early_out_contact_separating_velocity,
                            math::Vec3f const &gravitational_velocity_delta,
                            float max_separating_velocity_for_bounce) {
    for (auto i = 0; i != min_position_iterations_per_contact *
                              (int)_island_contacts.size();
         ++i) {
      if (auto const contact =
              _island_contacts.find_contact_of_least_separation(0.0f)) {
        std::visit([this](auto &&arg) { resolve_contact_position(arg); },
                   *contact);
      } else {
        break;
      }
    }
    for (auto i = 0; i != (max_position_iterations_per_contact -
                           min_position_iterations_per_contact) *
                              (int)_island_contacts.size();
         ++i) {
      if (auto const contact =
              _island_contacts.find_contact_of_least_separation(
                  early_out_contact_separation)) {
        std::visit([this](auto &&arg) { resolve_contact_position(arg); },
                   *contact);
      } else {
        break;
      }
    }
    for (auto i = 0; i != min_velocity_iterations_per_contact *
                              (int)_island_contacts.size();
         ++i) {
      if (auto const contact =
              _island_contacts.find_contact_of_least_separating_velocity(
                  0.0f)) {
        std::visit(
            [=, this](auto &&arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (std::is_same_v<T, Particle_static_body_contact *> ||
                            std::is_same_v<T,
                                           Rigid_body_static_body_contact *>) {
                resolve_contact_velocity(arg,
                                         gravitational_velocity_delta,
                                         max_separating_velocity_for_bounce);
              } else {
                resolve_contact_velocity(arg,
                                         max_separating_velocity_for_bounce);
              }
            },
            *contact);
      } else {
        break;
      }
    }
    for (auto i = 0; i != (max_velocity_iterations_per_contact -
                           min_velocity_iterations_per_contact) *
                              (int)_island_contacts.size();
         ++i) {
      if (auto const contact =
              _island_contacts.find_contact_of_least_separating_velocity(
                  early_out_contact_separating_velocity)) {
        std::visit(
            [=, this](auto &&arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr (std::is_same_v<T, Particle_static_body_contact *> ||
                            std::is_same_v<T,
                                           Rigid_body_static_body_contact *>) {
                resolve_contact_velocity(arg,
                                         gravitational_velocity_delta,
                                         max_separating_velocity_for_bounce);
              } else {
                resolve_contact_velocity(arg,
                                         max_separating_velocity_for_bounce);
              }
            },
            *contact);
      } else {
        break;
      }
    }
  }

  struct Positional_constraint_problem {
    math::Vec3f direction;
    float distance;
    std::array<math::Vec3f, 2> position;
    std::array<float, 2> inverse_mass;
    std::array<math::Mat3x3f, 2> inverse_inertia_tensor;
  };

  struct Positional_constraint_solution {
    std::array<math::Vec3f, 2> delta_position;
    std::array<math::Vec3f, 2> delta_orientation;
    float lambda;
  };

  Positional_constraint_solution
  solve_positional_constraint(Positional_constraint_problem const &problem) {
    auto const angular_impulse_per_impulse =
        std::array<Vec3f, 2>{cross(problem.position[0], problem.direction),
                             cross(problem.position[1], problem.direction)};
    auto const angular_displacement_per_impulse = std::array<Vec3f, 2>{
        problem.inverse_inertia_tensor[0] * angular_impulse_per_impulse[0],
        problem.inverse_inertia_tensor[1] * angular_impulse_per_impulse[1]};
    auto const rotational_displacement_per_impulse = std::array<Vec3f, 2>{
        cross(angular_displacement_per_impulse[0], problem.position[0]),
        cross(angular_displacement_per_impulse[1], problem.position[1])};
    auto const correcting_rotational_displacement_per_impulse =
        std::array<float, 2>{
            dot(rotational_displacement_per_impulse[0], problem.direction),
            dot(rotational_displacement_per_impulse[1], problem.direction)};
    auto const correcting_translational_displacement_per_impulse =
        std::array<float, 2>{problem.inverse_mass[0], problem.inverse_mass[1]};
    auto const correcting_displacement_per_impulse =
        correcting_translational_displacement_per_impulse[0] +
        correcting_translational_displacement_per_impulse[1] +
        correcting_rotational_displacement_per_impulse[0] +
        correcting_rotational_displacement_per_impulse[1];
    auto const impulse_per_correcting_displacement =
        1.0f / correcting_displacement_per_impulse;
    auto const impulse = problem.distance * impulse_per_correcting_displacement;
    auto correcting_translational_displacement = std::array<float, 2>{
        impulse * correcting_translational_displacement_per_impulse[0],
        impulse * -correcting_translational_displacement_per_impulse[1]};
    auto correcting_rotational_displacement = std::array<float, 2>{
        impulse * correcting_rotational_displacement_per_impulse[0],
        impulse * -correcting_rotational_displacement_per_impulse[1]};
    auto const max_rotational_displacement = std::array<float, 2>{
        max_rotational_displacement_coefficient * length(problem.position[0]),
        max_rotational_displacement_coefficient * length(problem.position[1])};
    for (auto i = 0; i != 2; ++i) {
      if (std::abs(correcting_rotational_displacement[i]) >
          max_rotational_displacement[i]) {
        auto const total_displacement =
            correcting_translational_displacement[i] +
            correcting_rotational_displacement[i];
        correcting_rotational_displacement[i] =
            std::signbit(correcting_rotational_displacement[i])
                ? -max_rotational_displacement[i]
                : max_rotational_displacement[i];
        correcting_translational_displacement[i] =
            total_displacement - correcting_rotational_displacement[i];
      }
    }
    auto const delta_position = std::array<math::Vec3f, 2>{
        problem.direction * correcting_translational_displacement[0],
        problem.direction * correcting_translational_displacement[1],
    };
    auto const angular_displacement_per_correcting_rotational_displacement =
        std::array<Vec3f, 2>{
            angular_displacement_per_impulse[0] /
                correcting_rotational_displacement_per_impulse[0],
            angular_displacement_per_impulse[1] /
                correcting_rotational_displacement_per_impulse[1]};
    auto const delta_orientation = std::array<Vec3f, 2>{
        angular_displacement_per_correcting_rotational_displacement[0] *
            correcting_rotational_displacement[0],
        angular_displacement_per_correcting_rotational_displacement[1] *
            correcting_rotational_displacement[1]};
    return {.delta_position = delta_position,
            .delta_orientation = delta_orientation,
            .lambda = impulse};
  }

  void apply_positional_correction(Particle_handle particle_handle,
                                   math::Vec3f const &delta_position) noexcept {
    auto const particle_data = _particles.data(particle_handle);
    particle_data->position += delta_position;
  }

  void
  apply_positional_correction(Dynamic_rigid_body_handle body_handle,
                              math::Vec3f const &delta_position,
                              math::Vec3f const &delta_orientation) noexcept {
    auto const body_data = _rigid_bodies.data(body_handle);
    body_data->position += delta_position;
    body_data->orientation +=
        0.5f * math::Quatf{0.0f, delta_orientation} * body_data->orientation;
    body_data->orientation = math::normalize(body_data->orientation);
  }

  void resolve_contact_position(Particle_particle_contact *contact) {
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
      update_contact_separations(particles[i], particle_position_deltas[i]);
    }
  }

  void resolve_contact_position(Particle_rigid_body_contact *contact) {
    auto const particle_data = _particles.data(contact->particle);
    auto const body_data = _rigid_bodies.data(contact->body);
    auto const rotation = Mat3x3f::rotation(body_data->orientation);
    auto const inverse_rotation = transpose(rotation);
    auto const inverse_inertia_tensor =
        rotation * body_data->inverse_inertia_tensor * inverse_rotation;
    auto const separation_solution = solve_positional_constraint({
        .direction = contact->normal,
        .distance = -contact->separation,
        .position = {Vec3f::zero(), contact->relative_position},
        .inverse_mass = {particle_data->inverse_mass, body_data->inverse_mass},
        .inverse_inertia_tensor = {Mat3x3f::zero(), inverse_inertia_tensor},
    });
    auto const contact_movement =
        (particle_data->position - particle_data->previous_position) -
        ((body_data->position + contact->relative_position) -
         (body_data->previous_position +
          Mat3x3f::rotation(body_data->previous_orientation) *
              inverse_rotation * contact->relative_position));
    auto const tangential_contact_movement =
        perp_unit(contact_movement, contact->normal);
    auto delta_position = separation_solution.delta_position;
    auto delta_orientation = separation_solution.delta_orientation[1];
    if (tangential_contact_movement != Vec3f::zero()) {
      auto const correction_distance = length(tangential_contact_movement);
      auto const correction_direction =
          tangential_contact_movement / -correction_distance;
      auto const friction_solution = solve_positional_constraint({
          .direction = correction_direction,
          .distance = correction_distance,
          .position = {Vec3f::zero(), contact->relative_position},
          .inverse_mass = {particle_data->inverse_mass,
                           body_data->inverse_mass},
          .inverse_inertia_tensor = {Mat3x3f::zero(), inverse_inertia_tensor},
      });
      auto const static_friction_coefficient =
          0.5f * (particle_data->material.static_friction_coefficient +
                  body_data->material.static_friction_coefficient);
      if (friction_solution.lambda <
          static_friction_coefficient * separation_solution.lambda) {
        delta_position[0] += friction_solution.delta_position[0];
        delta_position[1] += friction_solution.delta_position[1];
        delta_orientation += friction_solution.delta_orientation[1];
      }
    }
    apply_positional_correction(contact->particle, delta_position[0]);
    apply_positional_correction(
        contact->body, delta_position[1], delta_orientation);
    update_contact_separations(contact->particle, delta_position[0]);
    update_contact_separations(
        contact->body, delta_position[1], delta_orientation);
  }

  void resolve_contact_position(Particle_static_body_contact *contact) {
    auto const particle_data = _particles.data(contact->particle);
    auto const body_data = _static_bodies.data(contact->body);
    auto const separation_solution = solve_positional_constraint({
        .direction = contact->normal,
        .distance = -contact->separation,
        .position = {Vec3f::zero(), Vec3f::zero()},
        .inverse_mass = {particle_data->inverse_mass, 0.0f},
        .inverse_inertia_tensor = {Mat3x3f::zero(), Mat3x3f::zero()},
    });
    auto const contact_movement =
        particle_data->position - particle_data->previous_position;
    auto const tangential_contact_movement =
        perp_unit(contact_movement, contact->normal);
    auto delta_position = separation_solution.delta_position[0];
    if (tangential_contact_movement != Vec3f::zero()) {
      auto const correction_distance = length(tangential_contact_movement);
      auto const correction_direction =
          tangential_contact_movement / -correction_distance;
      auto const friction_solution = solve_positional_constraint({
          .direction = correction_direction,
          .distance = correction_distance,
          .position = {Vec3f::zero(), Vec3f::zero()},
          .inverse_mass = {particle_data->inverse_mass, 0.0f},
          .inverse_inertia_tensor = {Mat3x3f::zero(), Mat3x3f::zero()},
      });
      auto const static_friction_coefficient =
          0.5f * (particle_data->material.static_friction_coefficient +
                  body_data->material.static_friction_coefficient);
      if (friction_solution.lambda <
          static_friction_coefficient * separation_solution.lambda) {
        delta_position += friction_solution.delta_position[0];
      }
    }
    apply_positional_correction(contact->particle, delta_position);
    update_contact_separations(contact->particle, delta_position);
  }

  void resolve_contact_position(Rigid_body_rigid_body_contact *contact) {
    auto const data = std::array<Dynamic_rigid_body_data *, 2>{
        _rigid_bodies.data(contact->bodies[0]),
        _rigid_bodies.data(contact->bodies[1])};
    auto const rotation =
        std::array<Mat3x3f, 2>{Mat3x3f::rotation(data[0]->orientation),
                               Mat3x3f::rotation(data[1]->orientation)};
    auto const inverse_rotation =
        std::array<Mat3x3f, 2>{transpose(rotation[0]), transpose(rotation[1])};
    auto const inverse_inertia_tensor = std::array<Mat3x3f, 2>{
        rotation[0] * data[0]->inverse_inertia_tensor * inverse_rotation[0],
        rotation[1] * data[1]->inverse_inertia_tensor * inverse_rotation[1]};
    auto const separation_solution = solve_positional_constraint(
        {.direction = contact->normal,
         .distance = -contact->separation,
         .position = contact->relative_positions,
         .inverse_mass = {data[0]->inverse_mass, data[1]->inverse_mass},
         .inverse_inertia_tensor = inverse_inertia_tensor});
    auto const relative_contact_movement =
        ((data[0]->position + contact->relative_positions[0]) -
         (data[0]->previous_position +
          Mat3x3f::rotation(data[0]->previous_orientation) *
              inverse_rotation[0] * contact->relative_positions[0])) -
        ((data[1]->position + contact->relative_positions[1]) -
         (data[1]->previous_position +
          Mat3x3f::rotation(data[1]->previous_orientation) *
              inverse_rotation[1] * contact->relative_positions[1]));
    auto const tangential_relative_contact_movement =
        perp_unit(relative_contact_movement, contact->normal);
    auto delta_position = separation_solution.delta_position;
    auto delta_orientation = separation_solution.delta_orientation;
    if (tangential_relative_contact_movement != math::Vec3f::zero()) {
      auto const correction_distance =
          math::length(tangential_relative_contact_movement);
      auto const correction_direction =
          tangential_relative_contact_movement / -correction_distance;
      auto const friction_solution = solve_positional_constraint(
          {.direction = correction_direction,
           .distance = correction_distance,
           .position = contact->relative_positions,
           .inverse_mass = {data[0]->inverse_mass, data[1]->inverse_mass},
           .inverse_inertia_tensor = inverse_inertia_tensor});
      auto const static_friction_coefficient =
          0.5f * (data[0]->material.static_friction_coefficient +
                  data[1]->material.static_friction_coefficient);
      if (friction_solution.lambda <
          static_friction_coefficient * separation_solution.lambda) {
        for (auto i = 0; i != 2; ++i) {
          delta_position[i] += friction_solution.delta_position[i];
          delta_orientation[i] += friction_solution.delta_orientation[i];
        }
      }
    }
    for (auto i = 0; i != 2; ++i) {
      apply_positional_correction(
          contact->bodies[i], delta_position[i], delta_orientation[i]);
      update_contact_separations(
          contact->bodies[i], delta_position[i], delta_orientation[i]);
    }
  }

  void resolve_contact_position(Rigid_body_static_body_contact *contact) {
    auto const dynamic_body_data = _rigid_bodies.data(contact->rigid_body);
    auto const static_body_data = _static_bodies.data(contact->static_body);
    auto const rotation = Mat3x3f::rotation(dynamic_body_data->orientation);
    auto const inverse_rotation = transpose(rotation);
    auto const inverse_inertia_tensor =
        rotation * dynamic_body_data->inverse_inertia_tensor * inverse_rotation;
    auto const separation_solution = solve_positional_constraint(
        {.direction = contact->normal,
         .distance = -contact->separation,
         .position = {contact->relative_position, Vec3f::zero()},
         .inverse_mass = {dynamic_body_data->inverse_mass, 0.0f},
         .inverse_inertia_tensor = {inverse_inertia_tensor, Mat3x3f::zero()}});
    auto const contact_movement =
        (dynamic_body_data->position + contact->relative_position) -
        (dynamic_body_data->previous_position +
         Mat3x3f::rotation(dynamic_body_data->previous_orientation) *
             inverse_rotation * contact->relative_position);
    auto const tangential_contact_movement =
        perp_unit(contact_movement, contact->normal);
    auto delta_position = separation_solution.delta_position[0];
    auto delta_orientation = separation_solution.delta_orientation[0];
    if (tangential_contact_movement != Vec3f::zero()) {
      auto const correction_distance = length(tangential_contact_movement);
      auto const correction_direction =
          tangential_contact_movement / -correction_distance;
      auto const friction_solution = solve_positional_constraint(
          {.direction = correction_direction,
           .distance = correction_distance,
           .position = {contact->relative_position, Vec3f::zero()},
           .inverse_mass = {dynamic_body_data->inverse_mass, 0.0f},
           .inverse_inertia_tensor = {inverse_inertia_tensor,
                                      Mat3x3f::zero()}});
      auto const static_friction_coefficient =
          0.5f * (dynamic_body_data->material.static_friction_coefficient +
                  static_body_data->material.static_friction_coefficient);
      if (friction_solution.lambda <
          static_friction_coefficient * separation_solution.lambda) {
        delta_position += friction_solution.delta_position[0];
        delta_orientation += friction_solution.delta_orientation[0];
      }
    }
    apply_positional_correction(
        contact->rigid_body, delta_position, delta_orientation);
    update_contact_separations(
        contact->rigid_body, delta_position, delta_orientation);
  }

  void update_contact_separations(Particle_handle particle,
                                  Vec3f const &position_delta) {
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
      auto const delta = (contact->particles[0] == particle ? 1.0f : -1.0f) *
                         math::dot(position_delta, contact->normal);
      contact->separation += delta;
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      auto const delta = math::dot(position_delta, contact->normal);
      contact->separation += delta;
    }
    for (auto const contact : static_rigid_body_contacts) {
      auto const delta = math::dot(position_delta, contact->normal);
      contact->separation += delta;
    }
  }

  void update_contact_separations(Dynamic_rigid_body_handle body,
                                  math::Vec3f const &position_delta,
                                  math::Vec3f const &orientation_delta) {
    auto const data = _rigid_bodies.data(body);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      auto const delta =
          -math::dot(position_delta + math::cross(orientation_delta,
                                                  contact->relative_position),
                     contact->normal);
      contact->separation += delta;
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      if (contact->bodies[0] == body) {
        auto const delta = math::dot(
            position_delta +
                math::cross(orientation_delta, contact->relative_positions[0]),
            contact->normal);
        contact->separation += delta;
      } else {
        auto const delta = -math::dot(
            position_delta +
                math::cross(orientation_delta, contact->relative_positions[1]),
            contact->normal);
        contact->separation += delta;
      }
    }
    for (auto const contact : static_rigid_body_contacts) {
      auto const delta =
          math::dot(position_delta + math::cross(orientation_delta,
                                                 contact->relative_position),
                    contact->normal);
      contact->separation += delta;
    }
  }

  void resolve_contact_velocity(Particle_particle_contact *contact,
                                float max_separating_velocity_for_bounce) {
    auto const particles = contact->particles;
    auto const particle_datas = std::array<Particle_data *, 2>{
        _particles.data(particles[0]), _particles.data(particles[1])};
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const velocity_per_impulse =
        particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
    auto const impulse_per_velocity = 1.0f / velocity_per_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_datas[0]->material.restitution_coefficient +
                      particle_datas[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const separating_impulse_length =
        delta_separating_velocity * impulse_per_velocity;
    auto const separating_impulse = separating_impulse_length * normal;
    auto const frictional_impulse = [&]() {
      auto const relative_velocity =
          particle_datas[0]->velocity - particle_datas[1]->velocity;
      auto const sliding_velocity =
          relative_velocity - separating_velocity * normal;
      auto const sliding_speed_squared = math::length_squared(sliding_velocity);
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const static_friction_coefficient =
            0.5f * (particle_datas[0]->material.static_friction_coefficient +
                    particle_datas[1]->material.static_friction_coefficient);
        auto const dynamic_friction_coefficient =
            0.5f * (particle_datas[0]->material.dynamic_friction_coefficient +
                    particle_datas[1]->material.dynamic_friction_coefficient);
        auto const max_static_frictional_impulse_length =
            static_friction_coefficient * separating_impulse_length;
        auto const max_dynamic_frictional_impulse_length =
            dynamic_friction_coefficient * separating_impulse_length;
        auto const stopping_impulse_length =
            impulse_per_velocity * sliding_speed;
        if (stopping_impulse_length <= max_static_frictional_impulse_length) {
          return stopping_impulse_length * frictional_impulse_direction;
        } else {
          return max_dynamic_frictional_impulse_length *
                 frictional_impulse_direction;
        }
      } else {
        return math::Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
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

  void resolve_contact_velocity(Particle_rigid_body_contact *contact,
                                float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _rigid_bodies.data(body);
    auto const body_relative_contact_position = contact->relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const body_rotation = math::Mat3x3f::rotation(body_data->orientation);
    auto const body_inverse_rotation = math::transpose(body_rotation);
    auto const body_inverse_inertia_tensor = body_rotation *
                                             body_data->inverse_inertia_tensor *
                                             body_inverse_rotation;
    auto const body_angular_impulse_per_separating_impulse =
        math::cross(body_relative_contact_position, normal);
    auto const body_angular_velocity_per_separating_impulse =
        body_inverse_inertia_tensor *
        body_angular_impulse_per_separating_impulse;
    auto const separating_velocity_per_separating_impulse =
        particle_data->inverse_mass + body_data->inverse_mass +
        math::dot(math::cross(body_angular_velocity_per_separating_impulse,
                              body_relative_contact_position),
                  normal);
    auto const separating_impulse_per_separating_velocity =
        1.0f / separating_velocity_per_separating_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_data->material.restitution_coefficient +
                      body_data->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const separating_impulse_length =
        delta_separating_velocity * separating_impulse_per_separating_velocity;
    auto const separating_impulse = separating_impulse_length * normal;
    auto const relative_velocity =
        particle_data->velocity -
        (body_data->velocity + math::cross(body_data->angular_velocity,
                                           body_relative_contact_position));
    auto const sliding_velocity =
        relative_velocity - separating_velocity * normal;
    auto const sliding_speed_squared = math::length_squared(sliding_velocity);
    auto const frictional_impulse = [&]() {
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const body_angular_impulse_per_frictional_impulse = math::cross(
            body_relative_contact_position, frictional_impulse_direction);
        auto const body_angular_velocity_per_frictional_impulse =
            body_inverse_inertia_tensor *
            body_angular_impulse_per_frictional_impulse;
        auto const sliding_velocity_per_frictional_impulse =
            particle_data->inverse_mass + body_data->inverse_mass +
            math::dot(math::cross(body_angular_velocity_per_frictional_impulse,
                                  body_relative_contact_position),
                      frictional_impulse_direction);
        auto const frictional_impulse_per_sliding_velocity =
            1.0f / sliding_velocity_per_frictional_impulse;
        auto const static_friction_coefficient =
            0.5f * (particle_data->material.static_friction_coefficient +
                    body_data->material.static_friction_coefficient);
        auto const dynamic_friction_coefficient =
            0.5f * (particle_data->material.dynamic_friction_coefficient +
                    body_data->material.dynamic_friction_coefficient);
        auto const max_static_friction_impulse_length =
            static_friction_coefficient * separating_impulse_length;
        auto const max_dynamic_friction_impulse_length =
            dynamic_friction_coefficient * separating_impulse_length;
        auto const stopping_impulse_length =
            frictional_impulse_per_sliding_velocity * sliding_speed;
        if (stopping_impulse_length <= max_static_friction_impulse_length) {
          return stopping_impulse_length * frictional_impulse_direction;
        } else {
          return max_dynamic_friction_impulse_length *
                 frictional_impulse_direction;
        }
      } else {
        return math::Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
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

  void resolve_contact_velocity(Particle_static_body_contact *contact,
                                math::Vec3f const &gravitational_velocity_delta,
                                float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _static_bodies.data(body);
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
    auto const relative_velocity = particle_data->velocity;
    auto const sliding_velocity =
        relative_velocity - separating_velocity * normal;
    auto const sliding_velocity_length_squared =
        math::length_squared(sliding_velocity);
    auto const static_friction_coefficient =
        0.5f * (particle_data->material.static_friction_coefficient +
                body_data->material.static_friction_coefficient);
    auto const dynamic_friction_coefficient =
        0.5f * (particle_data->material.dynamic_friction_coefficient +
                body_data->material.dynamic_friction_coefficient);
    auto const max_static_friction_delta_sliding_velocity =
        static_friction_coefficient * delta_separating_velocity;
    auto const max_static_friction_delta_sliding_velocity_squared =
        max_static_friction_delta_sliding_velocity *
        max_static_friction_delta_sliding_velocity;
    auto const max_dynamic_friction_delta_sliding_velocity =
        dynamic_friction_coefficient * delta_separating_velocity;
    auto const delta_sliding_velocity =
        sliding_velocity_length_squared != 0.0f
            ? (sliding_velocity_length_squared <=
                       max_static_friction_delta_sliding_velocity_squared
                   ? -sliding_velocity
                   : -sliding_velocity *
                         max_dynamic_friction_delta_sliding_velocity /
                         std::sqrt(sliding_velocity_length_squared))
            : math::Vec3f::zero();
    auto const particle_velocity_delta =
        delta_separating_velocity * normal + delta_sliding_velocity;
    particle_data->velocity += particle_velocity_delta;
    update_particle_contact_separating_velocities(particle,
                                                  particle_velocity_delta);
  }

  void resolve_contact_velocity(Rigid_body_rigid_body_contact *contact,
                                float max_separating_velocity_for_bounce) {
    auto const bodies = contact->bodies;
    auto const body_datas = std::array<Dynamic_rigid_body_data *, 2>{
        _rigid_bodies.data(bodies[0]), _rigid_bodies.data(bodies[1])};
    auto const relative_contact_positions = contact->relative_positions;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const rotations = std::array<math::Mat3x3f, 2>{
        math::Mat3x3f::rotation(body_datas[0]->orientation),
        math::Mat3x3f::rotation(body_datas[1]->orientation)};
    auto const inverse_rotations = std::array<math::Mat3x3f, 2>{
        transpose(rotations[0]), transpose(rotations[1])};
    auto const inverse_inertia_tensors = std::array<math::Mat3x3f, 2>{
        rotations[0] * body_datas[0]->inverse_inertia_tensor *
            inverse_rotations[0],
        rotations[1] * body_datas[1]->inverse_inertia_tensor *
            inverse_rotations[1]};
    auto const angular_impulses_per_separating_impulse =
        std::array<Vec3f, 2>{cross(relative_contact_positions[0], normal),
                             cross(relative_contact_positions[1], normal)};
    auto const angular_velocities_per_separating_impulse = std::array<Vec3f, 2>{
        inverse_inertia_tensors[0] * angular_impulses_per_separating_impulse[0],
        inverse_inertia_tensors[1] *
            angular_impulses_per_separating_impulse[1]};
    auto const separating_velocity_per_separating_impulse =
        body_datas[0]->inverse_mass + body_datas[1]->inverse_mass +
        dot(cross(angular_velocities_per_separating_impulse[0],
                  relative_contact_positions[0]) +
                cross(angular_velocities_per_separating_impulse[1],
                      relative_contact_positions[1]),
            normal);
    auto const separating_impulse_per_separating_velocity =
        1.0f / separating_velocity_per_separating_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (body_datas[0]->material.restitution_coefficient +
                      body_datas[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const separating_impulse_length =
        delta_separating_velocity * separating_impulse_per_separating_velocity;
    auto const separating_impulse = separating_impulse_length * normal;
    auto const frictional_impulse = [&]() {
      auto const relative_velocity =
          (body_datas[0]->velocity +
           math::cross(body_datas[0]->angular_velocity,
                       relative_contact_positions[0])) -
          (body_datas[1]->velocity +
           math::cross(body_datas[1]->angular_velocity,
                       relative_contact_positions[1]));
      auto const sliding_velocity =
          relative_velocity - separating_velocity * normal;
      auto const sliding_speed_squared = length_squared(sliding_velocity);
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const angular_impulses_per_frictional_impulse =
            std::array<Vec3f, 2>{cross(relative_contact_positions[0],
                                       frictional_impulse_direction),
                                 cross(relative_contact_positions[1],
                                       frictional_impulse_direction)};
        auto const angular_velocities_per_frictional_impulse =
            std::array<Vec3f, 2>{
                inverse_inertia_tensors[0] *
                    angular_impulses_per_frictional_impulse[0],
                inverse_inertia_tensors[1] *
                    angular_impulses_per_frictional_impulse[1]};
        auto const sliding_velocity_per_frictional_impulse =
            body_datas[0]->inverse_mass + body_datas[1]->inverse_mass +
            dot(cross(angular_velocities_per_frictional_impulse[0],
                      relative_contact_positions[0]),
                cross(angular_velocities_per_frictional_impulse[1],
                      relative_contact_positions[1]));
        auto const frictional_impulse_per_sliding_velocity =
            1.0f / sliding_velocity_per_frictional_impulse;
        auto const static_friction_coefficient =
            0.5f * (body_datas[0]->material.static_friction_coefficient +
                    body_datas[1]->material.static_friction_coefficient);
        auto const dynamic_friction_coefficient =
            0.5f * (body_datas[0]->material.dynamic_friction_coefficient +
                    body_datas[1]->material.dynamic_friction_coefficient);
        auto const max_static_friction_impulse_length =
            static_friction_coefficient * separating_impulse_length;
        auto const max_dynamic_friction_impulse_length =
            dynamic_friction_coefficient * separating_impulse_length;
        auto const stopping_impulse_length =
            frictional_impulse_per_sliding_velocity * sliding_speed;
        if (stopping_impulse_length <= max_static_friction_impulse_length) {
          return stopping_impulse_length * frictional_impulse_direction;
        } else {
          return max_dynamic_friction_impulse_length *
                 frictional_impulse_direction;
        }
      } else {
        return Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
    auto const body_velocity_deltas =
        std::array<Vec3f, 2>{impulse * body_datas[0]->inverse_mass,
                             -impulse * body_datas[1]->inverse_mass};
    auto const body_angular_velocity_deltas = std::array<Vec3f, 2>{
        inverse_inertia_tensors[0] *
            cross(relative_contact_positions[0], impulse),
        inverse_inertia_tensors[1] *
            cross(relative_contact_positions[1], -impulse)};
    for (auto i = 0; i != 2; ++i) {
      body_datas[i]->velocity += body_velocity_deltas[i];
      body_datas[i]->angular_velocity += body_angular_velocity_deltas[i];
      update_dynamic_rigid_body_contact_separating_velocities(
          bodies[i], body_velocity_deltas[i], body_angular_velocity_deltas[i]);
    }
  }

  void resolve_contact_velocity(Rigid_body_static_body_contact *contact,
                                math::Vec3f const &gravitational_velocity_delta,
                                float max_separating_velocity_for_bounce) {
    auto const dynamic_body = contact->rigid_body;
    auto const dynamic_body_data = _rigid_bodies.data(dynamic_body);
    auto const static_body = contact->static_body;
    auto const static_body_data = _static_bodies.data(static_body);
    auto const dynamic_body_relative_contact_position =
        contact->relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity = contact->separating_velocity;
    auto const dynamic_body_rotation =
        math::Mat3x3f::rotation(dynamic_body_data->orientation);
    auto const dynamic_body_inverse_rotation = transpose(dynamic_body_rotation);
    auto const dynamic_body_inverse_inertia_tensor =
        dynamic_body_rotation * dynamic_body_data->inverse_inertia_tensor *
        dynamic_body_inverse_rotation;
    auto const angular_impulse_per_separating_impulse =
        cross(dynamic_body_relative_contact_position, normal);
    auto const angular_velocity_per_separating_impulse =
        dynamic_body_inverse_inertia_tensor *
        angular_impulse_per_separating_impulse;
    auto const separating_velocity_per_separating_impulse =
        dynamic_body_data->inverse_mass +
        dot(cross(angular_velocity_per_separating_impulse,
                  dynamic_body_relative_contact_position),
            normal);
    auto const separating_impulse_per_separating_velocity =
        1.0f / separating_velocity_per_separating_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (static_body_data->material.restitution_coefficient +
                      dynamic_body_data->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity -
        std::min(separating_velocity -
                     dot(gravitational_velocity_delta, normal),
                 0.0f) *
            restitution_coefficient;
    auto const separating_impulse_length =
        delta_separating_velocity * separating_impulse_per_separating_velocity;
    auto const separating_impulse = separating_impulse_length * normal;
    auto const frictional_impulse = [&]() {
      auto const relative_velocity =
          dynamic_body_data->velocity +
          math::cross(dynamic_body_data->angular_velocity,
                      dynamic_body_relative_contact_position);
      auto const sliding_velocity =
          relative_velocity - separating_velocity * normal;
      auto const sliding_speed_squared = length_squared(sliding_velocity);
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const angular_impulse_per_frictional_impulse =
            cross(dynamic_body_relative_contact_position,
                  frictional_impulse_direction);
        auto const angular_velocity_per_frictional_impulse =
            dynamic_body_inverse_inertia_tensor *
            angular_impulse_per_frictional_impulse;
        auto const sliding_velocity_per_frictional_impulse =
            dynamic_body_data->inverse_mass +
            dot(cross(angular_velocity_per_frictional_impulse,
                      dynamic_body_relative_contact_position),
                frictional_impulse_direction);
        auto const frictional_impulse_per_sliding_velocity =
            1.0f / sliding_velocity_per_frictional_impulse;
        auto const static_friction_coefficient =
            0.5f * (dynamic_body_data->material.static_friction_coefficient +
                    static_body_data->material.static_friction_coefficient);
        auto const dynamic_friction_coefficient =
            0.5f * (dynamic_body_data->material.dynamic_friction_coefficient +
                    static_body_data->material.dynamic_friction_coefficient);
        auto const max_static_friction_impulse_length =
            static_friction_coefficient * separating_impulse_length;
        auto const max_dynamic_friction_impulse_length =
            dynamic_friction_coefficient * separating_impulse_length;
        auto const stopping_impulse_length =
            frictional_impulse_per_sliding_velocity * sliding_speed;
        if (stopping_impulse_length <= max_static_friction_impulse_length) {
          return stopping_impulse_length * frictional_impulse_direction;
        } else {
          return max_dynamic_friction_impulse_length *
                 frictional_impulse_direction;
        }
      } else {
        return Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
    auto const dynamic_body_velocity_delta =
        dynamic_body_data->inverse_mass * impulse;
    auto const dynamic_body_angular_velocity_delta =
        dynamic_body_inverse_inertia_tensor *
        math::cross(dynamic_body_relative_contact_position, impulse);
    dynamic_body_data->velocity += dynamic_body_velocity_delta;
    dynamic_body_data->angular_velocity += dynamic_body_angular_velocity_delta;
    update_dynamic_rigid_body_contact_separating_velocities(
        dynamic_body,
        dynamic_body_velocity_delta,
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
      auto const delta = (contact->particles[0] == particle ? 1.0f : -1.0f) *
                         math::dot(velocity_delta, contact->normal);
      contact->separating_velocity += delta;
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      auto const delta = math::dot(velocity_delta, contact->normal);
      contact->separating_velocity += delta;
    }
    for (auto const contact : static_rigid_body_contacts) {
      auto const delta = math::dot(velocity_delta, contact->normal);
      contact->separating_velocity += delta;
    }
  }

  void update_dynamic_rigid_body_contact_separating_velocities(
      Dynamic_rigid_body_handle body,
      math::Vec3f const &velocity_delta,
      math::Vec3f const &angular_velocity_delta) {
    auto const data = _rigid_bodies.data(body);
    auto const particle_contacts =
        std::span{data->particle_contacts, data->particle_contact_count};
    auto const dynamic_rigid_body_contacts =
        std::span{data->dynamic_rigid_body_contacts,
                  data->dynamic_rigid_body_contact_count};
    auto const static_rigid_body_contacts =
        std::span{data->static_rigid_body_contacts,
                  data->static_rigid_body_contact_count};
    for (auto const contact : particle_contacts) {
      auto const delta =
          -math::dot(velocity_delta + math::cross(angular_velocity_delta,
                                                  contact->relative_position),
                     contact->normal);
      contact->separating_velocity += delta;
    }
    for (auto const contact : dynamic_rigid_body_contacts) {
      if (contact->bodies[0] == body) {
        auto const delta = math::dot(
            velocity_delta + math::cross(angular_velocity_delta,
                                         contact->relative_positions[0]),
            contact->normal);
        contact->separating_velocity += delta;
      } else {
        auto const delta = -math::dot(
            velocity_delta + math::cross(angular_velocity_delta,
                                         contact->relative_positions[1]),
            contact->normal);
        contact->separating_velocity += delta;
      }
    }
    for (auto const contact : static_rigid_body_contacts) {
      auto const delta =
          math::dot(velocity_delta + math::cross(angular_velocity_delta,
                                                 contact->relative_position),
                    contact->normal);
      contact->separating_velocity += delta;
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
    _rigid_bodies.for_each(
        [&](Dynamic_rigid_body_handle handle, Dynamic_rigid_body_data *data) {
          if (data->motion_callback != nullptr) {
            data->motion_callback->on_dynamic_rigid_body_motion(
                {handle, data->position, data->orientation});
          }
        });
  }

  std::unique_ptr<std::byte[]> _system_allocation;
  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  Particle_storage _particles;
  Static_rigid_body_storage _static_bodies;
  Rigid_body_storage _rigid_bodies;
  Array<std::pair<Particle_handle, Particle_handle>>
      _close_particle_particle_pairs;
  Array<std::pair<Particle_handle, Dynamic_rigid_body_handle>>
      _close_particle_rigid_body_pairs;
  Array<std::pair<Particle_handle, Static_rigid_body_handle>>
      _close_particle_static_body_pairs;
  Array<std::pair<Dynamic_rigid_body_handle, Dynamic_rigid_body_handle>>
      _close_rigid_body_rigid_body_pairs;
  Array<std::pair<Dynamic_rigid_body_handle, Static_rigid_body_handle>>
      _close_rigid_body_static_body_pairs;
  Array<Particle_particle_contact> _particle_particle_contacts;
  Array<Particle_rigid_body_contact> _particle_rigid_body_contacts;
  Array<Particle_static_body_contact> _particle_static_body_contacts;
  Array<Rigid_body_rigid_body_contact> _rigid_body_rigid_body_contacts;
  Array<Rigid_body_static_body_contact> _rigid_body_static_body_contacts;
  Array<Particle_particle_contact *> _particle_particle_contact_pointers;
  Array<Particle_rigid_body_contact *> _particle_rigid_body_contact_pointers;
  Array<Particle_static_body_contact *> _particle_static_body_contact_pointers;
  Array<Rigid_body_rigid_body_contact *>
      _rigid_body_rigid_body_contact_pointers;
  Array<Rigid_body_static_body_contact *>
      _rigid_body_static_body_contact_pointers;
  Object_stack _island_fringe;
  Contact_stack _island_contacts;
  Contact_cache _contact_cache;
  math::Vec3f _gravitational_acceleration;
};

Space::Space(Space_create_info const &create_info)
    : _impl{[&]() {
        auto const aabb_tree_memory_requirement =
            decltype(Impl::_aabb_tree)::memory_requirement(
                create_info.max_aabb_tree_leaf_nodes,
                create_info.max_aabb_tree_internal_nodes);
        auto const close_particle_particle_pairs_memory_requirement =
            decltype(Impl::_close_particle_particle_pairs)::memory_requirement(
                create_info.max_particle_particle_contacts);
        auto const close_particle_rigid_body_pairs_memory_requirement =
            decltype(Impl::_close_particle_rigid_body_pairs)::
                memory_requirement(
                    create_info.max_particle_rigid_body_contacts);
        auto const close_particle_static_body_pairs_memory_requirement =
            decltype(Impl::_close_particle_static_body_pairs)::
                memory_requirement(
                    create_info.max_particle_static_body_contacts);
        auto const close_rigid_body_rigid_body_pairs_memory_requirement =
            decltype(Impl::_close_rigid_body_rigid_body_pairs)::
                memory_requirement(
                    create_info.max_rigid_body_rigid_body_contacts);
        auto const close_rigid_body_static_body_pairs_memory_requirement =
            decltype(Impl::_close_rigid_body_static_body_pairs)::
                memory_requirement(
                    create_info.max_rigid_body_static_body_contacts);
        auto const particle_particle_contacts_memory_requirement =
            decltype(Impl::_particle_particle_contacts)::memory_requirement(
                create_info.max_particle_particle_contacts);
        auto const particle_rigid_body_contacts_memory_requirement =
            decltype(Impl::_particle_rigid_body_contacts)::memory_requirement(
                create_info.max_particle_rigid_body_contacts);
        auto const particle_static_body_contacts_memory_requirement =
            decltype(Impl::_particle_static_body_contacts)::memory_requirement(
                create_info.max_particle_static_body_contacts);
        auto const rigid_body_rigid_body_contacts_memory_requirement =
            decltype(Impl::_rigid_body_rigid_body_contacts)::memory_requirement(
                create_info.max_rigid_body_rigid_body_contacts);
        auto const rigid_body_static_body_contacts_memory_requirement =
            decltype(Impl::_rigid_body_static_body_contacts)::
                memory_requirement(
                    create_info.max_rigid_body_static_body_contacts);
        auto const particle_particle_contact_pointers_memory_requirement =
            decltype(Impl::_particle_particle_contact_pointers)::
                memory_requirement(2 *
                                   create_info.max_particle_particle_contacts);
        auto const particle_rigid_body_contact_pointers_memory_requirement =
            decltype(Impl::_particle_rigid_body_contact_pointers)::
                memory_requirement(
                    2 * create_info.max_particle_rigid_body_contacts);
        auto const particle_static_body_contact_pointers_memory_requirement =
            decltype(Impl::_particle_static_body_contact_pointers)::
                memory_requirement(
                    create_info.max_particle_static_body_contacts);
        auto const rigid_body_rigid_body_contact_pointers_memory_requirement =
            decltype(Impl::_rigid_body_rigid_body_contact_pointers)::
                memory_requirement(
                    2 * create_info.max_rigid_body_rigid_body_contacts);
        auto const rigid_body_static_body_contact_pointers_memory_requirement =
            decltype(Impl::_rigid_body_static_body_contact_pointers)::
                memory_requirement(
                    create_info.max_rigid_body_static_body_contacts);
        auto const island_fringe_memory_requirement =
            decltype(Impl::_island_fringe)::memory_requirement(
                create_info.max_island_object_count);
        auto const island_contacts_memory_requirement =
            decltype(Impl::_island_contacts)::memory_requirement(
                create_info.max_island_contact_count);
        auto const contact_cache_memory_requirement =
            decltype(Impl::_contact_cache)::memory_requirement(
                create_info.max_rigid_body_rigid_body_contacts,
                create_info.max_rigid_body_static_body_contacts);
        auto const total_memory_requirement =
            Stack_allocator<>::memory_requirement(
                {aabb_tree_memory_requirement,
                 close_particle_particle_pairs_memory_requirement,
                 close_particle_rigid_body_pairs_memory_requirement,
                 close_particle_static_body_pairs_memory_requirement,
                 close_rigid_body_rigid_body_pairs_memory_requirement,
                 close_rigid_body_static_body_pairs_memory_requirement,
                 particle_particle_contacts_memory_requirement,
                 particle_rigid_body_contacts_memory_requirement,
                 particle_static_body_contacts_memory_requirement,
                 rigid_body_rigid_body_contacts_memory_requirement,
                 rigid_body_static_body_contacts_memory_requirement,
                 particle_particle_contact_pointers_memory_requirement,
                 particle_rigid_body_contact_pointers_memory_requirement,
                 particle_static_body_contact_pointers_memory_requirement,
                 rigid_body_rigid_body_contact_pointers_memory_requirement,
                 rigid_body_static_body_contact_pointers_memory_requirement,
                 island_fringe_memory_requirement,
                 island_contacts_memory_requirement,
                 contact_cache_memory_requirement});
        auto system_allocation =
            std::make_unique<std::byte[]>(total_memory_requirement);
        auto allocator = Stack_allocator{
            make_block(system_allocation.get(), total_memory_requirement)};
        return std::make_unique<Impl>(
            create_info,
            std::move(system_allocation),
            allocator.alloc(aabb_tree_memory_requirement).begin,
            allocator.alloc(close_particle_particle_pairs_memory_requirement)
                .begin,
            allocator.alloc(close_particle_rigid_body_pairs_memory_requirement)
                .begin,
            allocator.alloc(close_particle_static_body_pairs_memory_requirement)
                .begin,
            allocator
                .alloc(close_rigid_body_rigid_body_pairs_memory_requirement)
                .begin,
            allocator
                .alloc(close_rigid_body_static_body_pairs_memory_requirement)
                .begin,
            allocator.alloc(particle_particle_contacts_memory_requirement)
                .begin,
            allocator.alloc(particle_rigid_body_contacts_memory_requirement)
                .begin,
            allocator.alloc(particle_static_body_contacts_memory_requirement)
                .begin,
            allocator.alloc(rigid_body_rigid_body_contacts_memory_requirement)
                .begin,
            allocator.alloc(rigid_body_static_body_contacts_memory_requirement)
                .begin,
            allocator
                .alloc(particle_particle_contact_pointers_memory_requirement)
                .begin,
            allocator
                .alloc(particle_rigid_body_contact_pointers_memory_requirement)
                .begin,
            allocator
                .alloc(particle_static_body_contact_pointers_memory_requirement)
                .begin,
            allocator
                .alloc(
                    rigid_body_rigid_body_contact_pointers_memory_requirement)
                .begin,
            allocator
                .alloc(
                    rigid_body_static_body_contact_pointers_memory_requirement)
                .begin,
            allocator.alloc(island_fringe_memory_requirement).begin,
            allocator.alloc(island_contacts_memory_requirement).begin,
            allocator.alloc(contact_cache_memory_requirement).begin);
      }()} {}

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