#include "world.h"

#include <cstdint>

#include <iostream>
#include <latch>

#include "../util/list.h"
#include "../util/map.h"
#include "aabb_tree.h"

using marlon::math::Mat3x3f;
using marlon::math::Mat3x4f;
using marlon::math::Quatf;
using marlon::math::Vec3f;

namespace marlon {
namespace physics {
using util::Block;
using util::List;
using util::Map;
using util::Stack_allocator;

using util::make_block;

namespace {
using Aabb_tree_payload_t =
    std::variant<Particle_handle, Static_body_handle, Rigid_body_handle>;

enum class Object_type { particle, rigid_body, static_body };

enum class Contact_type {
  particle_particle,
  particle_rigid_body,
  particle_static_body,
  rigid_body_rigid_body,
  rigid_body_static_body
};

struct Contact {
  Vec3f normal;
  float separation;
  // float separating_velocity;

  Contact() = default;

  constexpr Contact(Vec3f const &normal,
                    float separation/*,
                    float separating_velocity*/) noexcept
      : normal{normal},
        separation{separation}/*,
        separating_velocity{separating_velocity}*/ {}
};

struct Particle_particle_contact : Contact {
  std::array<Particle_handle, 2> particles;

  Particle_particle_contact() = default;

  constexpr Particle_particle_contact(
      Vec3f const &normal,
      float separation,
      // float separating_velocity,
      std::array<Particle_handle, 2> const &particles) noexcept
      : Contact{normal, separation /*, separating_velocity*/},
        particles{particles} {}
};

struct Particle_rigid_body_contact : Contact {
  Particle_handle particle;
  Rigid_body_handle body;
  Vec3f relative_position;

  Particle_rigid_body_contact() = default;

  constexpr Particle_rigid_body_contact(
      Vec3f const &normal,
      float separation,
      // float separating_velocity,
      Particle_handle particle,
      Rigid_body_handle body,
      Vec3f const &body_relative_position) noexcept
      : Contact{normal, separation /*, separating_velocity*/},
        particle{particle},
        body{body},
        relative_position{body_relative_position} {}
};

struct Particle_static_body_contact : Contact {
  Particle_handle particle;
  Static_body_handle body;

  Particle_static_body_contact() = default;

  constexpr Particle_static_body_contact(Vec3f const &normal,
                                         float separation,
                                         //  float separating_velocity,
                                         Particle_handle particle,
                                         Static_body_handle body)
      : Contact{normal, separation /*, separating_velocity*/},
        particle{particle},
        body{body} {}
};

struct Rigid_body_rigid_body_contact : Contact {
  std::array<Rigid_body_handle, 2> bodies;
  std::array<Vec3f, 2> relative_positions;

  Rigid_body_rigid_body_contact() = default;

  constexpr Rigid_body_rigid_body_contact(
      Vec3f const &normal,
      float separation,
      // float separating_velocity,
      std::array<Rigid_body_handle, 2> const &bodies,
      std::array<Vec3f, 2> const &relative_positions) noexcept
      : Contact{normal, separation /*, separating_velocity*/},
        bodies{bodies},
        relative_positions{relative_positions} {}
};

struct Rigid_body_static_body_contact : Contact {
  Rigid_body_handle rigid_body;
  Static_body_handle static_body;
  Vec3f relative_position;

  Rigid_body_static_body_contact() = default;

  constexpr Rigid_body_static_body_contact(
      Vec3f const &normal,
      float separation,
      // float separating_velocity,
      Rigid_body_handle dynamic_body,
      Static_body_handle static_body,
      Vec3f const &relative_position) noexcept
      : Contact{normal, separation /*, separating_velocity*/},
        rigid_body{dynamic_body},
        static_body{static_body},
        relative_position{relative_position} {}
};

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node{};
  Particle_handle *particle_neighbors{};
  Rigid_body_handle *rigid_body_neighbors{};
  Static_body_handle *static_body_neighbors{};
  Particle_particle_contact **particle_contacts{};
  Particle_rigid_body_contact **rigid_body_contacts{};
  Particle_static_body_contact **static_body_contacts{};
  Particle_motion_callback *motion_callback{};
  float radius{};
  float inverse_mass{};
  Material material{};
  Vec3f previous_position{};
  Vec3f position{};
  Vec3f velocity{};
  float waking_motion{};
  std::uint16_t particle_neighbor_count{};
  std::uint16_t rigid_body_neighbor_count{};
  std::uint16_t static_body_neighbor_count{};
  std::uint16_t particle_contact_count{};
  std::uint16_t rigid_body_contact_count{};
  std::uint16_t static_body_contact_count{};
  bool marked{};
  bool visited{};
  bool awake{};
};

struct Rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node{};
  Particle_handle *particle_neighbors{};
  Rigid_body_handle *rigid_body_neighbors{};
  Static_body_handle *static_body_neighbors{};
  Particle_rigid_body_contact **particle_contacts{};
  Rigid_body_static_body_contact **static_body_contacts{};
  Rigid_body_rigid_body_contact **rigid_body_contacts{};
  Rigid_body_motion_callback *motion_callback{};
  Shape shape;
  float inverse_mass{};
  Mat3x3f inverse_inertia_tensor{};
  Material material{};
  Vec3f previous_position{};
  Vec3f position{};
  Vec3f velocity{};
  Quatf previous_orientation{};
  Quatf orientation{};
  Vec3f angular_velocity{};
  float waking_motion;
  std::uint16_t particle_neighbor_count{};
  std::uint16_t rigid_body_neighbor_count{};
  std::uint16_t static_body_neighbor_count{};
  std::uint16_t particle_contact_count{};
  std::uint16_t rigid_body_contact_count{};
  std::uint16_t static_body_contact_count{};
  bool marked{};
  bool visited{};
  bool awake{};
};

struct Static_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node;
  Shape shape;
  Material material;
  Mat3x4f transform;
  Mat3x4f inverse_transform;
};

class Particle_storage {
public:
  explicit Particle_storage(std::size_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size * sizeof(Particle_data))},
        _free_indices(size),
        _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
    }
  }

  Particle_handle create(Particle_data const &data) {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for particles"};
    }
    auto const index = _free_indices.back();
    new (_data.get() + sizeof(Particle_data) * index) Particle_data{data};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return Particle_handle{index};
  }

  void destroy(Particle_handle particle) {
    _free_indices.emplace_back(particle.value);
    _occupancy_bits[particle.value] = false;
  }

  Particle_data const *data(Particle_handle handle) const noexcept {
    return std::launder(reinterpret_cast<Particle_data const *>(
        _data.get() + sizeof(Particle_data) * handle.value));
  }

  Particle_data *data(Particle_handle handle) noexcept {
    return std::launder(reinterpret_cast<Particle_data *>(
        _data.get() + sizeof(Particle_data) * handle.value));
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
      : _data{std::make_unique<std::byte[]>(size * sizeof(Rigid_body_data))},
        _free_indices(size),
        _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
    }
  }

  Rigid_body_handle create(Rigid_body_data const &data) {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for static rigid bodies"};
    }
    auto const index = _free_indices.back();
    new (_data.get() + sizeof(Rigid_body_data) * index) Rigid_body_data{data};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return Rigid_body_handle{index};
  }

  void destroy(Rigid_body_handle rigid_body) {
    _free_indices.emplace_back(rigid_body.value);
    _occupancy_bits[rigid_body.value] = false;
  }

  Rigid_body_data const *data(Rigid_body_handle handle) const noexcept {
    return std::launder(reinterpret_cast<Rigid_body_data const *>(
        _data.get() + sizeof(Rigid_body_data) * handle.value));
  }

  Rigid_body_data *data(Rigid_body_handle handle) noexcept {
    return std::launder(reinterpret_cast<Rigid_body_data *>(
        _data.get() + sizeof(Rigid_body_data) * handle.value));
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits[i]) {
        auto const handle = Rigid_body_handle{static_cast<std::uint32_t>(i)};
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

class Static_body_storage {
public:
  explicit Static_body_storage(std::size_t size) noexcept
      : _data{std::make_unique<std::byte[]>(size * sizeof(Static_body_data))},
        _free_indices(size),
        _occupancy_bits(size) {
    for (auto i = std::size_t{}; i != size; ++i) {
      _free_indices[i] = static_cast<std::uint32_t>(size - i - 1);
    }
  }

  Static_body_handle create(Static_body_data const &data) {
    if (_free_indices.empty()) {
      throw std::runtime_error{"Out of space for static rigid bodies"};
    }
    auto const index = _free_indices.back();
    new (_data.get() + sizeof(Static_body_data) * index) Static_body_data{data};
    _free_indices.pop_back();
    _occupancy_bits[index] = true;
    return Static_body_handle{index};
  }

  void destroy(Static_body_handle static_body) {
    _free_indices.emplace_back(static_body.value);
    _occupancy_bits[static_body.value] = false;
  }

  Static_body_data const *data(Static_body_handle handle) const noexcept {
    return std::launder(reinterpret_cast<Static_body_data const *>(
        _data.get() + sizeof(Static_body_data) * handle.value));
  }

  Static_body_data *data(Static_body_handle handle) noexcept {
    return std::launder(reinterpret_cast<Static_body_data *>(
        _data.get() + sizeof(Static_body_data) * handle.value));
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n; ++i) {
      if (_occupancy_bits[i]) {
        auto const handle = Static_body_handle{static_cast<std::uint32_t>(i)};
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

class Dynamic_object_list {
  using Allocator = Stack_allocator<>;

public:
  static constexpr std::size_t
  memory_requirement(std::size_t max_size) noexcept {
    return Allocator::memory_requirement(
        {decltype(_object_types)::memory_requirement(max_size),
         decltype(_object_handles)::memory_requirement(max_size)});
  }

  constexpr Dynamic_object_list() noexcept = default;

  explicit Dynamic_object_list(Block block, std::size_t max_size) noexcept
      : Dynamic_object_list{block.begin, max_size} {}

  explicit Dynamic_object_list(void *block_begin,
                               std::size_t max_size) noexcept {
    auto allocator =
        Allocator{make_block(block_begin, memory_requirement(max_size))};
    _object_types = make_list<Object_type>(allocator, max_size).second;
    _object_handles = make_list<Object_handle>(allocator, max_size).second;
  }

  std::variant<Particle_handle, Rigid_body_handle>
  at(std::size_t i) const noexcept {
    auto const object_type = _object_types[i];
    auto const object_handle = _object_handles[i];
    switch (object_type) {
    case Object_type::particle:
      return Particle_handle{object_handle};
    case Object_type::rigid_body:
      return Rigid_body_handle{object_handle};
    case Object_type::static_body:
    default:
      math::unreachable();
    }
  }

  std::variant<Particle_handle, Rigid_body_handle> front() const noexcept {
    assert(!empty());
    return at(0);
  }

  std::variant<Particle_handle, Rigid_body_handle> back() const noexcept {
    assert(&_object_types.back() == &_object_types[size() - 1]);
    return at(size() - 1);
  }

  bool empty() const noexcept { return _object_types.empty(); }

  std::size_t size() const noexcept { return _object_types.size(); }

  std::size_t max_size() const noexcept { return _object_types.max_size(); }

  std::size_t capacity() const noexcept { return _object_types.capacity(); }

  void clear() noexcept {
    _object_types.clear();
    _object_handles.clear();
  }

  void push_back(Particle_handle h) {
    _object_types.push_back(Object_type::particle);
    _object_handles.push_back(h.value);
  }

  void push_back(Rigid_body_handle h) {
    _object_types.push_back(Object_type::rigid_body);
    _object_handles.push_back(h.value);
  }

  void push_back(std::variant<Particle_handle, Rigid_body_handle> h) {
    std::visit([this](auto &&arg) { push_back(arg); }, h);
  }

  void pop_back() {
    _object_types.pop_back();
    _object_handles.pop_back();
  }

private:
  List<Object_type> _object_types;
  List<Object_handle> _object_handles;
};

template <typename Allocator>
std::pair<Block, Dynamic_object_list>
make_dynamic_object_list(Allocator &allocator, std::size_t max_size) {
  auto const block =
      allocator.alloc(Dynamic_object_list::memory_requirement(max_size));
  return {block, Dynamic_object_list{block, max_size}};
}

class Neighbor_group_storage {
  using Allocator = Stack_allocator<>;

public:
  static constexpr std::size_t memory_requirement(std::size_t max_object_count,
                                                  std::size_t max_group_count) {
    return Allocator::memory_requirement(
        {decltype(_objects)::memory_requirement(max_object_count),
         decltype(_groups)::memory_requirement(max_group_count)});
  }

  constexpr Neighbor_group_storage() noexcept = default;

  Neighbor_group_storage(Block block,
                         std::size_t max_object_count,
                         std::size_t max_group_count)
      : Neighbor_group_storage{block.begin, max_object_count, max_group_count} {
  }

  Neighbor_group_storage(void *block_begin,
                         std::size_t max_object_count,
                         std::size_t max_group_count) {
    auto allocator = Allocator{make_block(
        block_begin, memory_requirement(max_object_count, max_group_count))};
    _objects = make_dynamic_object_list(allocator, max_object_count).second;
    _groups = util::make_list<Group>(allocator, max_group_count).second;
  }

  std::size_t object_count() const noexcept { return _objects.size(); }

  std::variant<Particle_handle, Rigid_body_handle>
  object(std::size_t object_index) const noexcept {
    return _objects.at(object_index);
  }

  std::size_t group_count() const noexcept { return _groups.size(); }

  std::size_t group_begin(std::size_t group_index) const noexcept {
    return _groups[group_index].begin;
  }

  std::size_t group_end(std::size_t group_index) const noexcept {
    return _groups[group_index].end;
  }

  void clear() noexcept {
    _objects.clear();
    _groups.clear();
  }

  void begin_group() {
    auto const index = static_cast<std::uint32_t>(_objects.size());
    _groups.push_back({index, index});
  }

  void add_to_group(Particle_handle object) {
    _objects.push_back(object);
    ++_groups.back().end;
  }

  void add_to_group(Rigid_body_handle object) {
    _objects.push_back(object);
    ++_groups.back().end;
  }

  void add_to_group(std::variant<Particle_handle, Rigid_body_handle> object) {
    std::visit([this](auto &&arg) { add_to_group(arg); }, object);
  }

private:
  struct Group {
    std::uint32_t begin;
    std::uint32_t end;
  };

  Dynamic_object_list _objects;
  List<Group> _groups;
};

template <typename Allocator>
std::pair<Block, Neighbor_group_storage>
make_neighbor_group_storage(Allocator &allocator,
                            std::size_t max_object_count,
                            std::size_t max_group_count) {
  auto const block = allocator.alloc(Neighbor_group_storage::memory_requirement(
      max_object_count, max_group_count));
  return {block,
          Neighbor_group_storage{block, max_object_count, max_group_count}};
}

class Contact_list {
public:
  static constexpr std::size_t
  memory_requirement(std::size_t capacity) noexcept {
    return Stack_allocator<>::memory_requirement(
        {List<Contact_type>::memory_requirement(capacity),
         List<Contact *>::memory_requirement(capacity)});
  }

  constexpr Contact_list() noexcept = default;

  explicit Contact_list(Block block, std::size_t const capacity) noexcept
      : _impl{[&]() {
          auto allocator = Stack_allocator{block};
          auto const contact_types_block =
              allocator.alloc(List<Contact_type>::memory_requirement(capacity));
          auto const contacts_block =
              allocator.alloc(List<Contact *>::memory_requirement(capacity));
          return Impl{
              .contact_types =
                  List<Contact_type>{contact_types_block.begin, capacity},
              .contacts = List<Contact *>{contacts_block.begin, capacity}};
        }()} {}

  explicit Contact_list(void *block_begin, std::size_t const capacity) noexcept
      : Contact_list{make_block(block_begin, memory_requirement(capacity)),
                     capacity} {}

  bool empty() const noexcept { return _impl.contact_types.empty(); }

  std::size_t size() const noexcept { return _impl.contact_types.size(); }

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
    case Contact_type::particle_rigid_body:
      return static_cast<Particle_rigid_body_contact *>(result);
    case Contact_type::particle_static_body:
      return static_cast<Particle_static_body_contact *>(result);
    case Contact_type::rigid_body_rigid_body:
      return static_cast<Rigid_body_rigid_body_contact *>(result);
    case Contact_type::rigid_body_static_body:
      return static_cast<Rigid_body_static_body_contact *>(result);
    }
  }

  // std::optional<std::variant<Particle_particle_contact *,
  //                            Particle_rigid_body_contact *,
  //                            Particle_static_body_contact *,
  //                            Rigid_body_rigid_body_contact *,
  //                            Rigid_body_static_body_contact *>>
  // select_contact_position_solve(float max_separation,
  //                               std::size_t first,
  //                               std::size_t last) const noexcept {
  //   auto result = static_cast<Contact *const *>(nullptr);
  //   auto result_separation = max_separation;
  //   auto const contacts =
  //       std::span{_impl.contacts.data() + first, _impl.contacts.data() +
  //       last};
  //   for (auto const &contact : contacts) {
  //     if (contact->separation < result_separation) {
  //       result = &contact;
  //       result_separation = contact->separation;
  //     }
  //   }
  //   if (result) {
  //     auto const i = result - _impl.contacts.data();
  //     switch (_impl.contact_types[i]) {
  //     case Contact_type::particle_particle:
  //       return static_cast<Particle_particle_contact *>(*result);
  //     case Contact_type::particle_rigid_body:
  //       return static_cast<Particle_rigid_body_contact *>(*result);
  //     case Contact_type::particle_static_body:
  //       return static_cast<Particle_static_body_contact *>(*result);
  //     case Contact_type::rigid_body_rigid_body:
  //       return static_cast<Rigid_body_rigid_body_contact *>(*result);
  //     case Contact_type::rigid_body_static_body:
  //       return static_cast<Rigid_body_static_body_contact *>(*result);
  //     default:
  //       math::unreachable();
  //     }
  //   } else {
  //     return std::nullopt;
  //   }
  // }

  // std::optional<std::variant<Particle_particle_contact *,
  //                            Particle_rigid_body_contact *,
  //                            Particle_static_body_contact *,
  //                            Rigid_body_rigid_body_contact *,
  //                            Rigid_body_static_body_contact *>>
  // select_contact_velocity_solve(float max_separating_velocity,
  //                               std::size_t first,
  //                               std::size_t last) const noexcept {
  //   auto result = static_cast<Contact *const *>(nullptr);
  //   auto result_separating_velocity = max_separating_velocity;
  //   auto const contacts =
  //       std::span{_impl.contacts.data() + first, _impl.contacts.data() +
  //       last};
  //   for (auto const &contact : contacts) {
  //     if (contact->separating_velocity < result_separating_velocity) {
  //       result = &contact;
  //       result_separating_velocity = contact->separating_velocity;
  //     }
  //   }
  //   if (result) {
  //     auto const i = result - _impl.contacts.data();
  //     switch (_impl.contact_types[i]) {
  //     case Contact_type::particle_particle:
  //       return static_cast<Particle_particle_contact *>(*result);
  //     case Contact_type::particle_rigid_body:
  //       return static_cast<Particle_rigid_body_contact *>(*result);
  //     case Contact_type::particle_static_body:
  //       return static_cast<Particle_static_body_contact *>(*result);
  //     case Contact_type::rigid_body_rigid_body:
  //       return static_cast<Rigid_body_rigid_body_contact *>(*result);
  //     case Contact_type::rigid_body_static_body:
  //       return static_cast<Rigid_body_static_body_contact *>(*result);
  //     default:
  //       math::unreachable();
  //     }
  //   } else {
  //     return std::nullopt;
  //   }
  // }

  void clear() noexcept {
    _impl.contact_types.clear();
    _impl.contacts.clear();
  }

  void push_back(Particle_particle_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_particle);
    _impl.contacts.push_back(c);
  }

  void push_back(Particle_rigid_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_rigid_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Particle_static_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::particle_static_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Rigid_body_rigid_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::rigid_body_rigid_body);
    _impl.contacts.push_back(c);
  }

  void push_back(Rigid_body_static_body_contact *c) {
    _impl.contact_types.push_back(Contact_type::rigid_body_static_body);
    _impl.contacts.push_back(c);
  }

private:
  struct Impl {
    List<Contact_type> contact_types;
    List<Contact *> contacts;
  };

  Impl _impl{};
};

template <typename Allocator>
std::pair<Block, Contact_list> make_contact_list(Allocator &allocator,
                                                 std::size_t capacity) {
  auto const block =
      allocator.alloc(Contact_list::memory_requirement(capacity));
  return {block, Contact_list{block, capacity}};
}

// caching constants
// auto constexpr max_cached_contacts_per_object_pair = 4;
// auto constexpr max_cached_contact_drift = 1.0f / 128.0f;
// auto constexpr max_cached_contact_separation = 1.0f / 128.0f;

// integration constants
auto constexpr velocity_damping_factor = 0.99f;
auto constexpr waking_motion_epsilon = 1.0f / 32.0f;
auto constexpr waking_motion_initializer = 2.0f * waking_motion_epsilon;
auto constexpr waking_motion_limit = 8.0f * waking_motion_epsilon;
auto constexpr waking_motion_smoothing_factor = 7.0f / 8.0f;

// class Contact_cache {
// public:
//   static constexpr std::size_t
//   memory_requirement(std::size_t max_rigid_body_rigid_body_pairs,
//                      std::size_t max_rigid_body_static_body_pairs) {
//     return Stack_allocator<>::memory_requirement(
//         {decltype(_rigid_body_rigid_body_pairs)::memory_requirement(
//              max_rigid_body_rigid_body_pairs),
//          decltype(_rigid_body_static_body_pairs)::memory_requirement(
//              max_rigid_body_static_body_pairs)});
//   }

//   constexpr Contact_cache() = default;

//   explicit Contact_cache(Block block,
//                          std::size_t max_rigid_body_rigid_body_pairs,
//                          std::size_t max_rigid_body_static_body_pairs)
//       : Contact_cache{block.begin,
//                       max_rigid_body_rigid_body_pairs,
//                       max_rigid_body_static_body_pairs} {}

//   explicit Contact_cache(void *block_begin,
//                          std::size_t max_rigid_body_rigid_body_pairs,
//                          std::size_t max_rigid_body_static_body_pairs) {
//     auto allocator = Stack_allocator<>{
//         make_block(block_begin,
//                    memory_requirement(max_rigid_body_rigid_body_pairs,
//                                       max_rigid_body_static_body_pairs))};
//     _rigid_body_rigid_body_pairs =
//         make_map<std::uint64_t, Rigid_body_rigid_body_pair>(
//             allocator, max_rigid_body_rigid_body_pairs)
//             .second;
//     _rigid_body_static_body_pairs =
//         make_map<std::uint64_t, Rigid_body_static_body_pair>(
//             allocator, max_rigid_body_static_body_pairs)
//             .second;
//   }

//   std::span<Rigid_body_rigid_body_contact const> get_contacts(
//       std::pair<Rigid_body_handle, Rigid_body_handle> objects) const noexcept
//       {
//     if (objects.first.value > objects.second.value) {
//       std::swap(objects.first, objects.second);
//     }
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     auto const it = _rigid_body_rigid_body_pairs.find(key);
//     if (it != _rigid_body_rigid_body_pairs.end()) {
//       return it->second.get_contacts();
//     } else {
//       return {};
//     }
//   }

//   std::span<Rigid_body_rigid_body_contact> get_contacts(
//       std::pair<Rigid_body_handle, Rigid_body_handle> objects) noexcept {
//     if (objects.first.value > objects.second.value) {
//       std::swap(objects.first, objects.second);
//     }
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     auto const it = _rigid_body_rigid_body_pairs.find(key);
//     if (it != _rigid_body_rigid_body_pairs.end()) {
//       return it->second.get_contacts();
//     } else {
//       return {};
//     }
//   }

//   std::span<Rigid_body_static_body_contact const> get_contacts(
//       std::pair<Rigid_body_handle, Static_body_handle> objects) const
//       noexcept {
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     auto const it = _rigid_body_static_body_pairs.find(key);
//     if (it != _rigid_body_static_body_pairs.end()) {
//       return it->second.get_contacts();
//     } else {
//       return {};
//     }
//   }

//   std::span<Rigid_body_static_body_contact> get_contacts(
//       std::pair<Rigid_body_handle, Static_body_handle> objects) noexcept {
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     auto const it = _rigid_body_static_body_pairs.find(key);
//     if (it != _rigid_body_static_body_pairs.end()) {
//       return it->second.get_contacts();
//     } else {
//       return {};
//     }
//   }

//   void set_marked(std::pair<Rigid_body_handle, Rigid_body_handle> objects,
//                   bool marked = true) {
//     if (objects.first.value > objects.second.value) {
//       std::swap(objects.first, objects.second);
//     }
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      static_cast<std::uint64_t>(objects.second.value);
//     auto const it = _rigid_body_rigid_body_pairs.find(key);
//     if (it != _rigid_body_rigid_body_pairs.end()) {
//       it->second.set_marked(marked);
//     }
//   }

//   void set_marked(std::pair<Rigid_body_handle, Static_body_handle> objects,
//                   bool marked = true) {
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      static_cast<std::uint64_t>(objects.second.value);
//     auto const it = _rigid_body_static_body_pairs.find(key);
//     if (it != _rigid_body_static_body_pairs.end()) {
//       it->second.set_marked(marked);
//     }
//   }

//   void clear(std::pair<Rigid_body_handle, Rigid_body_handle> objects) {
//     if (objects.first.value > objects.second.value) {
//       std::swap(objects.first, objects.second);
//     }
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     _rigid_body_rigid_body_pairs.erase(key);
//   }

//   void clear(std::pair<Rigid_body_handle, Static_body_handle> objects) {
//     auto const key = (static_cast<std::uint64_t>(objects.first.value) << 32)
//     |
//                      objects.second.value;
//     _rigid_body_static_body_pairs.erase(key);
//   }

//   void clear_unmarked() {
//     for (auto it = _rigid_body_rigid_body_pairs.begin();
//          it != _rigid_body_rigid_body_pairs.end();) {
//       if (it->second.is_marked()) {
//         it->second.set_marked(false);
//         ++it;
//       } else {
//         it = _rigid_body_rigid_body_pairs.erase(it);
//       }
//     }
//     for (auto it = _rigid_body_static_body_pairs.begin();
//          it != _rigid_body_static_body_pairs.end();) {
//       if (it->second.is_marked()) {
//         it->second.set_marked(false);
//         ++it;
//       } else {
//         it = _rigid_body_static_body_pairs.erase(it);
//       }
//     }
//   }

//   std::size_t update(Rigid_body_rigid_body_contact contact,
//                      Rigid_body_storage const &storage) {
//     if (contact.bodies[0].value > contact.bodies[1].value) {
//       contact.normal = -contact.normal;
//       std::swap(contact.bodies[0], contact.bodies[1]);
//       std::swap(contact.relative_positions[0],
//       contact.relative_positions[1]);
//     }
//     auto const data = std::array<Rigid_body_data const *, 2>{
//         storage.data(contact.bodies[0]), storage.data(contact.bodies[1])};
//     auto const positions =
//         std::array<Vec3f, 2>{data[0]->position, data[1]->position};
//     auto const orientations =
//         std::array<Quatf, 2>{data[0]->orientation, data[1]->orientation};
//     auto const key =
//         (static_cast<std::uint64_t>(contact.bodies[0].value) << 32) |
//         contact.bodies[1].value;
//     auto const it = _rigid_body_rigid_body_pairs.find(key);
//     if (it != _rigid_body_rigid_body_pairs.end()) {
//       it->second.update(storage);
//       it->second.cache(contact, positions, orientations);
//       return it->second.get_contacts().size();
//     } else {
//       _rigid_body_rigid_body_pairs.emplace(
//           key, Rigid_body_rigid_body_pair{contact, positions, orientations});
//       return 1;
//     }
//   }

//   std::size_t update(Rigid_body_static_body_contact const &contact,
//                      Rigid_body_storage const &storage) {
//     auto const data = storage.data(contact.rigid_body);
//     auto const position = data->position;
//     auto const orientation = data->orientation;
//     auto const key =
//         (static_cast<std::uint64_t>(contact.rigid_body.value) << 32) |
//         contact.static_body.value;
//     auto const it = _rigid_body_static_body_pairs.find(key);
//     if (it != _rigid_body_static_body_pairs.end()) {
//       it->second.update(storage);
//       it->second.cache(contact, position, orientation);
//       return it->second.get_contacts().size();
//     } else {
//       _rigid_body_static_body_pairs.emplace(
//           key, Rigid_body_static_body_pair{contact, position, orientation});
//       return 1;
//     }
//   }

// private:
//   class Rigid_body_rigid_body_pair {
//   public:
//     Rigid_body_rigid_body_pair(Rigid_body_rigid_body_contact const &contact,
//                                std::array<Vec3f, 2> const &body_positions,
//                                std::array<Quatf, 2> const &body_orientations)
//         : _contacts{contact},
//           _contact_displacements{
//               body_positions[0] + contact.relative_positions[0] -
//               body_positions[1] - contact.relative_positions[1]},
//           _contact_body_orientations{body_orientations},
//           _size{1},
//           _marked{false},
//           _fresh{true} {}

//     std::span<Rigid_body_rigid_body_contact const>
//     get_contacts() const noexcept {
//       return {_contacts.data(), _size};
//     }

//     std::span<Rigid_body_rigid_body_contact> get_contacts() noexcept {
//       return {_contacts.data(), _size};
//     }

//     bool is_marked() const noexcept { return _marked; }

//     void set_marked(bool marked = true) noexcept { _marked = marked; }

//     void update(Rigid_body_storage const &rigid_bodies) noexcept {
//       if (_fresh && _size != 0) {
//         _fresh = false;
//         _size = 0;
//       } else {
//         if (_size != 0) {
//           auto const handles = _contacts[0].bodies;
//           auto const datas = std::array<Rigid_body_data const *, 2>{
//               rigid_bodies.data(handles[0]), rigid_bodies.data(handles[1])};
//           auto const delta_orientation_quaternions = std::array<Quatf, 2>{
//               normalize(datas[0]->orientation *
//                         conjugate(datas[0]->previous_orientation)),
//               normalize(datas[1]->orientation *
//                         conjugate(datas[1]->previous_orientation))};
//           auto const delta_orientations = std::array<Mat3x3f, 2>{
//               Mat3x3f::rotation(delta_orientation_quaternions[0]),
//               Mat3x3f::rotation(delta_orientation_quaternions[1])};
//           for (std::uint8_t i = 0; i < _size;) {
//             auto &contact = _contacts[i];
//             auto keep_contact = false;
//             auto const relative_positions = std::array<Vec3f, 2>{
//                 delta_orientations[0] * contact.relative_positions[0],
//                 delta_orientations[1] * contact.relative_positions[1]};
//             auto const absolute_positions = std::array<Vec3f, 2>{
//                 datas[0]->position + relative_positions[0],
//                 datas[1]->position + relative_positions[1]};
//             auto const displacement =
//                 absolute_positions[0] - absolute_positions[1];
//             auto const drift = displacement - _contact_displacements[i];
//             auto const drift_squared = length_squared(drift);
//             if (drift_squared <=
//                     max_cached_contact_drift * max_cached_contact_drift &&
//                 std::abs(normalize(datas[0]->orientation *
//                                    conjugate(_contact_body_orientations[i][0]))
//                              .w) > 0.999f &&
//                 std::abs(normalize(datas[1]->orientation *
//                                    conjugate(_contact_body_orientations[i][1]))
//                              .w) > 0.999f) {
//               contact.separation += dot(contact.normal, drift);
//               if (contact.separation <= max_cached_contact_separation) {
//                 contact.separating_velocity = dot(
//                     contact.normal,
//                     (datas[0]->velocity +
//                      cross(datas[0]->angular_velocity,
//                      relative_positions[0])) -
//                         (datas[1]->velocity +
//                         cross(datas[1]->angular_velocity,
//                                                     relative_positions[1])));
//                 contact.relative_positions = relative_positions;
//                 keep_contact = true;
//               }
//             }
//             if (keep_contact) {
//               ++i;
//             } else {
//               std::shift_left(
//                   &_contacts.data()[i], &_contacts.data()[_size], 1);
//               std::shift_left(&_contact_displacements.data()[i],
//                               &_contact_displacements.data()[_size],
//                               1);
//               std::shift_left(&_contact_body_orientations.data()[i],
//                               &_contact_body_orientations.data()[_size],
//                               1);
//               --_size;
//             }
//           }
//         }
//         if (_size == 0) {
//           _fresh = true;
//         }
//       }
//     }

//     void cache(Rigid_body_rigid_body_contact const &contact,
//                std::array<Vec3f, 2> const &body_positions,
//                std::array<Quatf, 2> const &body_orientations) {
//       for (std::uint8_t i = 0; i < _size; ++i) {
//         if (length_squared(_contacts[i].relative_positions[0] -
//                            contact.relative_positions[0]) <=
//                 max_cached_contact_drift * max_cached_contact_drift ||
//             length_squared(_contacts[i].relative_positions[1] -
//                            contact.relative_positions[1]) <=
//                 max_cached_contact_drift * max_cached_contact_drift) {
//           _contacts[i] = contact;
//           _contact_displacements[i] =
//               (body_positions[0] + contact.relative_positions[0]) -
//               (body_positions[1] + contact.relative_positions[1]);
//           _contact_body_orientations[i] = body_orientations;
//           return;
//         }
//       }
//       if (_size < max_cached_contacts_per_object_pair) {
//         _contacts[_size] = contact;
//         _contact_displacements[_size] =
//             (body_positions[0] + contact.relative_positions[0]) -
//             (body_positions[1] + contact.relative_positions[1]);
//         _contact_body_orientations[_size] = body_orientations;
//         ++_size;
//       } else {
//         auto const min_it = std::min_element(
//             _contacts.begin(),
//             _contacts.end(),
//             [&](auto const &lhs, auto const &rhs) {
//               return std::min(length_squared(lhs.relative_positions[0] -
//                                              contact.relative_positions[0]),
//                               length_squared(lhs.relative_positions[1] -
//                                              contact.relative_positions[1]))
//                                              <
//                      std::min(length_squared(rhs.relative_positions[0] -
//                                              contact.relative_positions[0]),
//                               length_squared(rhs.relative_positions[1] -
//                                              contact.relative_positions[1]));
//             });
//         auto const i = min_it - _contacts.begin();
//         _contacts[i] = contact;
//         _contact_displacements[i] =
//             (body_positions[0] + contact.relative_positions[0]) -
//             (body_positions[1] + contact.relative_positions[1]);
//         _contact_body_orientations[i] = body_orientations;
//       }
//     }

//   private:
//     std::array<Rigid_body_rigid_body_contact,
//                max_cached_contacts_per_object_pair>
//         _contacts;
//     std::array<Vec3f, max_cached_contacts_per_object_pair>
//         _contact_displacements;
//     std::array<std::array<Quatf, 2>, max_cached_contacts_per_object_pair>
//         _contact_body_orientations;
//     std::uint8_t _size;
//     bool _marked;
//     bool _fresh;
//   };

//   class Rigid_body_static_body_pair {
//   public:
//     Rigid_body_static_body_pair(Rigid_body_static_body_contact const
//     &contact,
//                                 Vec3f const &rigid_body_position,
//                                 Quatf const &rigid_body_orientation)
//         : _contacts{contact},
//           _contact_positions{rigid_body_position +
//           contact.relative_position},
//           _contact_body_orientations{rigid_body_orientation},
//           _size{1},
//           _marked{false},
//           _fresh{true} {}

//     std::span<Rigid_body_static_body_contact const>
//     get_contacts() const noexcept {
//       return {_contacts.data(), _size};
//     }

//     std::span<Rigid_body_static_body_contact> get_contacts() noexcept {
//       return {_contacts.data(), _size};
//     }

//     bool is_marked() const noexcept { return _marked; }

//     void set_marked(bool marked = true) noexcept { _marked = marked; }

//     void update(Rigid_body_storage const &rigid_bodies) noexcept {
//       if (_fresh && _size != 0) {
//         _fresh = false;
//         _size = 0;
//       } else {
//         if (_size != 0) {
//           auto const rigid_body_handle = _contacts[0].rigid_body;
//           auto const rigid_body_data = rigid_bodies.data(rigid_body_handle);
//           auto const delta_orientation_quaternion =
//               normalize(rigid_body_data->orientation *
//                         conjugate(rigid_body_data->previous_orientation));
//           auto const delta_orientation_matrix =
//               Mat3x3f::rotation(delta_orientation_quaternion);
//           for (std::uint8_t i = 0; i < _size;) {
//             auto &contact = _contacts[i];
//             auto keep_contact = false;
//             auto const relative_position =
//                 delta_orientation_matrix * contact.relative_position;
//             auto const absolute_position =
//                 rigid_body_data->position + relative_position;
//             auto const drift = absolute_position - _contact_positions[i];
//             auto const drift_squared = length_squared(drift);
//             if (drift_squared <=
//                     max_cached_contact_drift * max_cached_contact_drift &&
//                 std::abs(normalize(rigid_body_data->orientation *
//                                    conjugate(_contact_body_orientations[i]))
//                              .w) > 0.999f) {
//               contact.separation += dot(contact.normal, drift);
//               if (contact.separation <= max_cached_contact_separation) {
//                 contact.separating_velocity =
//                     dot(contact.normal,
//                         rigid_body_data->velocity +
//                             cross(rigid_body_data->angular_velocity,
//                                   relative_position));
//                 contact.relative_position = relative_position;
//                 keep_contact = true;
//               }
//             }
//             if (keep_contact) {
//               ++i;
//             } else {
//               std::shift_left(
//                   &_contacts.data()[i], &_contacts.data()[_size], 1);
//               std::shift_left(&_contact_positions.data()[i],
//                               &_contact_positions.data()[_size],
//                               1);
//               std::shift_left(&_contact_body_orientations.data()[i],
//                               &_contact_body_orientations.data()[_size],
//                               1);
//               --_size;
//             }
//           }
//         }
//         if (_size == 0) {
//           _fresh = true;
//         }
//       }
//     }

//     void cache(Rigid_body_static_body_contact const &contact,
//                Vec3f const &rigid_body_position,
//                Quatf const &rigid_body_orientation) {
//       for (std::uint8_t i = 0; i < _size; ++i) {
//         if (length_squared(_contacts[i].relative_position -
//                            contact.relative_position) <=
//             max_cached_contact_drift * max_cached_contact_drift) {
//           _contacts[i] = contact;
//           _contact_positions[i] =
//               rigid_body_position + contact.relative_position;
//           _contact_body_orientations[i] = rigid_body_orientation;
//           return;
//         }
//       }
//       if (_size < max_cached_contacts_per_object_pair) {
//         _contacts[_size] = contact;
//         _contact_positions[_size] =
//             rigid_body_position + contact.relative_position;
//         _contact_body_orientations[_size] = rigid_body_orientation;
//         ++_size;
//       } else {
//         auto const min_it = std::min_element(
//             _contacts.begin(),
//             _contacts.end(),
//             [&](auto const &lhs, auto const &rhs) {
//               return length_squared(lhs.relative_position -
//                                     contact.relative_position) <
//                      length_squared(rhs.relative_position -
//                                     contact.relative_position);
//             });
//         auto const i = min_it - _contacts.begin();
//         _contacts[i] = contact;
//         _contact_positions[i] = rigid_body_position +
//         contact.relative_position; _contact_body_orientations[i] =
//         rigid_body_orientation;
//       }
//     }

//   private:
//     std::array<Rigid_body_static_body_contact,
//                max_cached_contacts_per_object_pair>
//         _contacts;
//     std::array<Vec3f, max_cached_contacts_per_object_pair>
//     _contact_positions; std::array<Quatf,
//     max_cached_contacts_per_object_pair>
//         _contact_body_orientations;
//     std::uint8_t _size;
//     bool _marked;
//     bool _fresh;
//   };

//   util::Map<std::uint64_t, Rigid_body_rigid_body_pair>
//       _rigid_body_rigid_body_pairs;
//   util::Map<std::uint64_t, Rigid_body_static_body_pair>
//       _rigid_body_static_body_pairs;
// };

// template <typename Allocator>
// std::pair<Block, Contact_cache>
// make_contact_cache(Allocator &allocator,
//                    std::size_t max_rigid_body_rigid_body_pairs,
//                    std::size_t max_rigid_body_static_body_pairs) {
//   auto const block = allocator.alloc(Contact_cache::memory_requirement(
//       max_rigid_body_rigid_body_pairs, max_rigid_body_static_body_pairs));
//   return {block,
//           Contact_cache{block,
//                         max_rigid_body_rigid_body_pairs,
//                         max_rigid_body_static_body_pairs}};
// }

struct Positional_constraint_problem {
  Vec3f direction;
  float distance;
  std::array<Vec3f, 2> position;
  std::array<float, 2> inverse_mass;
  std::array<Mat3x3f, 2> inverse_inertia_tensor;
};

struct Positional_constraint_solution {
  std::array<Vec3f, 2> delta_position;
  std::array<Vec3f, 2> delta_orientation;
  float lambda;
};

auto constexpr max_rotational_displacement_factor = 0.2f;

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
      max_rotational_displacement_factor * length(problem.position[0]),
      max_rotational_displacement_factor * length(problem.position[1])};
  for (auto i = 0; i != 2; ++i) {
    if (std::abs(correcting_rotational_displacement[i]) >
        max_rotational_displacement[i]) {
      auto const total_displacement = correcting_translational_displacement[i] +
                                      correcting_rotational_displacement[i];
      correcting_rotational_displacement[i] =
          std::signbit(correcting_rotational_displacement[i])
              ? -max_rotational_displacement[i]
              : max_rotational_displacement[i];
      correcting_translational_displacement[i] =
          total_displacement - correcting_rotational_displacement[i];
    }
  }
  auto const delta_position = std::array<Vec3f, 2>{
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

class Contact_group_storage;

class Contact_group_solve_task : public util::Task {
public:
  struct Intrinsic_state {
    std::latch *latch;
    Particle_storage *particles;
    Rigid_body_storage *rigid_bodies;
    Static_body_storage *static_bodies;
    Contact_group_storage const *contact_groups;
    int min_desired_position_iterations_per_contact;
    int max_desired_position_iterations_per_contact;
    int max_position_iterations;
    int min_desired_velocity_iterations_per_contact;
    int max_desired_velocity_iterations_per_contact;
    int max_velocity_iterations;
    float early_out_contact_separation;
    float early_out_contact_separating_velocity;
    math::Vec3f gravitational_delta_velocity;
    float restitution_max_separating_velocity;
  };

  // constexpr Contact_group_solve_task(Intrinsic_state const *intrinsic_state,
  //                                    std::size_t group_index) noexcept
  //     : _intrinsic_state{intrinsic_state}, _group_index{group_index} {}

  // void run(unsigned) final;

  Intrinsic_state const *get_intrinsic_state() { return _intrinsic_state; }

  void set_instrinsic_state(Intrinsic_state const *intrinsic_state) noexcept {
    _intrinsic_state = intrinsic_state;
  }

private:
  //   void resolve_contact_position(Particle_particle_contact *contact) {
  //     auto const particles = contact->particles;
  //     auto const particle_datas = std::array<Particle_data *, 2>{
  //         _intrinsic_state->particles->data(particles[0]),
  //         _intrinsic_state->particles->data(particles[1]),
  //     };
  //     auto const distance_per_impulse =
  //         particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
  //     auto const impulse_per_distance = 1.0f / distance_per_impulse;
  //     auto const impulse =
  //         -contact->separation * impulse_per_distance * contact->normal;
  //     auto const particle_position_deltas =
  //         std::array<Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
  //                              impulse * -particle_datas[1]->inverse_mass};
  //     particle_datas[0]->position += particle_position_deltas[0];
  //     particle_datas[1]->position += particle_position_deltas[1];
  //     // for (auto i = 0; i != 2; ++i) {
  //     //   update_separations(particles[i], particle_position_deltas[i]);
  //     // }
  //   }

  //   void resolve_contact_position(Particle_rigid_body_contact *contact) {
  //     auto const particle_data =
  //         _intrinsic_state->particles->data(contact->particle);
  //     auto const body_data =
  //     _intrinsic_state->rigid_bodies->data(contact->body); auto const
  //     rotation = Mat3x3f::rotation(body_data->orientation); auto const
  //     inverse_rotation = transpose(rotation); auto const
  //     inverse_inertia_tensor =
  //         rotation * body_data->inverse_inertia_tensor * inverse_rotation;
  //     auto const separation_solution = solve_positional_constraint({
  //         .direction = contact->normal,
  //         .distance = -contact->separation,
  //         .position = {Vec3f::zero(), contact->relative_position},
  //         .inverse_mass = {particle_data->inverse_mass,
  //         body_data->inverse_mass}, .inverse_inertia_tensor =
  //         {Mat3x3f::zero(), inverse_inertia_tensor},
  //     });
  //     auto const contact_movement =
  //         (particle_data->position - particle_data->previous_position) -
  //         ((body_data->position + contact->relative_position) -
  //          (body_data->previous_position +
  //           Mat3x3f::rotation(body_data->previous_orientation) *
  //               inverse_rotation * contact->relative_position));
  //     auto const tangential_contact_movement =
  //         perp_unit(contact_movement, contact->normal);
  //     auto delta_position = separation_solution.delta_position;
  //     auto delta_orientation = separation_solution.delta_orientation[1];
  //     if (tangential_contact_movement != Vec3f::zero()) {
  //       auto const correction_distance = length(tangential_contact_movement);
  //       auto const correction_direction =
  //           tangential_contact_movement / -correction_distance;
  //       auto const friction_solution = solve_positional_constraint({
  //           .direction = correction_direction,
  //           .distance = correction_distance,
  //           .position = {Vec3f::zero(), contact->relative_position},
  //           .inverse_mass = {particle_data->inverse_mass,
  //                            body_data->inverse_mass},
  //           .inverse_inertia_tensor = {Mat3x3f::zero(),
  //           inverse_inertia_tensor},
  //       });
  //       auto const static_friction_coefficient =
  //           0.5f * (particle_data->material.static_friction_coefficient +
  //                   body_data->material.static_friction_coefficient);
  //       if (friction_solution.lambda <
  //           static_friction_coefficient * separation_solution.lambda) {
  //         delta_position[0] += friction_solution.delta_position[0];
  //         delta_position[1] += friction_solution.delta_position[1];
  //         delta_orientation += friction_solution.delta_orientation[1];
  //       }
  //     }
  //     apply_positional_correction(contact->particle, delta_position[0]);
  //     apply_positional_correction(
  //         contact->body, delta_position[1], delta_orientation);
  //     // update_separations(contact->particle, delta_position[0]);
  //     // update_separations(contact->body, delta_position[1],
  //     delta_orientation);
  //   }

  //   void resolve_contact_position(Particle_static_body_contact *contact) {
  //     auto const particle_data =
  //         _intrinsic_state->particles->data(contact->particle);
  //     auto const body_data =
  //     _intrinsic_state->static_bodies->data(contact->body); auto const
  //     separation_solution = solve_positional_constraint({
  //         .direction = contact->normal,
  //         .distance = -contact->separation,
  //         .position = {Vec3f::zero(), Vec3f::zero()},
  //         .inverse_mass = {particle_data->inverse_mass, 0.0f},
  //         .inverse_inertia_tensor = {Mat3x3f::zero(), Mat3x3f::zero()},
  //     });
  //     auto const contact_movement =
  //         particle_data->position - particle_data->previous_position;
  //     auto const tangential_contact_movement =
  //         perp_unit(contact_movement, contact->normal);
  //     auto delta_position = separation_solution.delta_position[0];
  //     if (tangential_contact_movement != Vec3f::zero()) {
  //       auto const correction_distance = length(tangential_contact_movement);
  //       auto const correction_direction =
  //           tangential_contact_movement / -correction_distance;
  //       auto const friction_solution = solve_positional_constraint({
  //           .direction = correction_direction,
  //           .distance = correction_distance,
  //           .position = {Vec3f::zero(), Vec3f::zero()},
  //           .inverse_mass = {particle_data->inverse_mass, 0.0f},
  //           .inverse_inertia_tensor = {Mat3x3f::zero(), Mat3x3f::zero()},
  //       });
  //       auto const static_friction_coefficient =
  //           0.5f * (particle_data->material.static_friction_coefficient +
  //                   body_data->material.static_friction_coefficient);
  //       if (friction_solution.lambda <
  //           static_friction_coefficient * separation_solution.lambda) {
  //         delta_position += friction_solution.delta_position[0];
  //       }
  //     }
  //     apply_positional_correction(contact->particle, delta_position);
  //     // update_separations(contact->particle, delta_position);
  //   }

  //   void resolve_contact_position(Rigid_body_rigid_body_contact *contact) {
  //     auto const data = std::array<Rigid_body_data *, 2>{
  //         _intrinsic_state->rigid_bodies->data(contact->bodies[0]),
  //         _intrinsic_state->rigid_bodies->data(contact->bodies[1]),
  //     };
  //     auto const rotation = std::array<Mat3x3f, 2>{
  //         Mat3x3f::rotation(data[0]->orientation),
  //         Mat3x3f::rotation(data[1]->orientation),
  //     };
  //     auto const inverse_rotation = std::array<Mat3x3f, 2>{
  //         transpose(rotation[0]),
  //         transpose(rotation[1]),
  //     };
  //     auto const inverse_inertia_tensor = std::array<Mat3x3f, 2>{
  //         rotation[0] * data[0]->inverse_inertia_tensor *
  //         inverse_rotation[0], rotation[1] * data[1]->inverse_inertia_tensor
  //         * inverse_rotation[1],
  //     };
  //     auto const separation_solution = solve_positional_constraint({
  //         .direction = contact->normal,
  //         .distance = -contact->separation,
  //         .position = contact->relative_positions,
  //         .inverse_mass = {data[0]->inverse_mass, data[1]->inverse_mass},
  //         .inverse_inertia_tensor = inverse_inertia_tensor,
  //     });
  //     auto const relative_contact_movement =
  //         ((data[0]->position + contact->relative_positions[0]) -
  //          (data[0]->previous_position +
  //           Mat3x3f::rotation(data[0]->previous_orientation) *
  //               inverse_rotation[0] * contact->relative_positions[0])) -
  //         ((data[1]->position + contact->relative_positions[1]) -
  //          (data[1]->previous_position +
  //           Mat3x3f::rotation(data[1]->previous_orientation) *
  //               inverse_rotation[1] * contact->relative_positions[1]));
  //     auto const tangential_relative_contact_movement =
  //         perp_unit(relative_contact_movement, contact->normal);
  //     auto delta_position = separation_solution.delta_position;
  //     auto delta_orientation = separation_solution.delta_orientation;
  //     if (tangential_relative_contact_movement != Vec3f::zero()) {
  //       auto const correction_distance =
  //           length(tangential_relative_contact_movement);
  //       auto const correction_direction =
  //           tangential_relative_contact_movement / -correction_distance;
  //       auto const friction_solution = solve_positional_constraint(
  //           {.direction = correction_direction,
  //            .distance = correction_distance,
  //            .position = contact->relative_positions,
  //            .inverse_mass = {data[0]->inverse_mass, data[1]->inverse_mass},
  //            .inverse_inertia_tensor = inverse_inertia_tensor});
  //       auto const static_friction_coefficient =
  //           0.5f * (data[0]->material.static_friction_coefficient +
  //                   data[1]->material.static_friction_coefficient);
  //       if (friction_solution.lambda <
  //           static_friction_coefficient * separation_solution.lambda) {
  //         for (auto i = 0; i != 2; ++i) {
  //           delta_position[i] += friction_solution.delta_position[i];
  //           delta_orientation[i] += friction_solution.delta_orientation[i];
  //         }
  //       }
  //     }
  //     for (auto i = 0; i != 2; ++i) {
  //       apply_positional_correction(
  //           contact->bodies[i], delta_position[i], delta_orientation[i]);
  //       // update_separations(
  //       //     contact->bodies[i], delta_position[i], delta_orientation[i]);
  //     }
  //   }

  //   void resolve_contact_position(Rigid_body_static_body_contact *contact) {
  //     auto const dynamic_body_data =
  //         _intrinsic_state->rigid_bodies->data(contact->rigid_body);
  //     auto const static_body_data =
  //         _intrinsic_state->static_bodies->data(contact->static_body);
  //     auto const rotation =
  //     Mat3x3f::rotation(dynamic_body_data->orientation); auto const
  //     inverse_rotation = transpose(rotation); auto const
  //     inverse_inertia_tensor =
  //         rotation * dynamic_body_data->inverse_inertia_tensor *
  //         inverse_rotation;
  //     auto const separation_solution = solve_positional_constraint(
  //         {.direction = contact->normal,
  //          .distance = -contact->separation,
  //          .position = {contact->relative_position, Vec3f::zero()},
  //          .inverse_mass = {dynamic_body_data->inverse_mass, 0.0f},
  //          .inverse_inertia_tensor = {inverse_inertia_tensor,
  //          Mat3x3f::zero()}});
  //     auto const contact_movement =
  //         (dynamic_body_data->position + contact->relative_position) -
  //         (dynamic_body_data->previous_position +
  //          Mat3x3f::rotation(dynamic_body_data->previous_orientation) *
  //              inverse_rotation * contact->relative_position);
  //     auto const tangential_contact_movement =
  //         perp_unit(contact_movement, contact->normal);
  //     auto delta_position = separation_solution.delta_position[0];
  //     auto delta_orientation = separation_solution.delta_orientation[0];
  //     if (tangential_contact_movement != Vec3f::zero()) {
  //       auto const correction_distance = length(tangential_contact_movement);
  //       auto const correction_direction =
  //           tangential_contact_movement / -correction_distance;
  //       auto const friction_solution = solve_positional_constraint(
  //           {.direction = correction_direction,
  //            .distance = correction_distance,
  //            .position = {contact->relative_position, Vec3f::zero()},
  //            .inverse_mass = {dynamic_body_data->inverse_mass, 0.0f},
  //            .inverse_inertia_tensor = {inverse_inertia_tensor,
  //                                       Mat3x3f::zero()}});
  //       auto const static_friction_coefficient =
  //           0.5f * (dynamic_body_data->material.static_friction_coefficient +
  //                   static_body_data->material.static_friction_coefficient);
  //       if (friction_solution.lambda <
  //           static_friction_coefficient * separation_solution.lambda) {
  //         delta_position += friction_solution.delta_position[0];
  //         delta_orientation += friction_solution.delta_orientation[0];
  //       }
  //     }
  //     apply_positional_correction(
  //         contact->rigid_body, delta_position, delta_orientation);
  //     // update_separations(contact->rigid_body, delta_position,
  //     // delta_orientation);
  //   }

  //   void apply_positional_correction(Particle_handle particle_handle,
  //                                    Vec3f const &delta_position) noexcept {
  //     auto const particle_data =
  //         _intrinsic_state->particles->data(particle_handle);
  //     particle_data->position += delta_position;
  //   }

  //   void apply_positional_correction(Rigid_body_handle body_handle,
  //                                    Vec3f const &delta_position,
  //                                    Vec3f const &delta_orientation) noexcept
  //                                    {
  //     auto const body_data =
  //     _intrinsic_state->rigid_bodies->data(body_handle); body_data->position
  //     += delta_position; body_data->orientation +=
  //         0.5f * Quatf{0.0f, delta_orientation} * body_data->orientation;
  //     body_data->orientation = normalize(body_data->orientation);
  //   }

  // void update_separations(Particle_handle particle,
  //                         Vec3f const &position_delta) {
  //   auto const data = _intrinsic_state->particles->data(particle);
  //   auto const particle_contacts =
  //       std::span{data->particle_contacts, data->particle_contact_count};
  //   auto const dynamic_rigid_body_contacts =
  //       std::span{data->rigid_body_contacts, data->rigid_body_contact_count};
  //   auto const static_rigid_body_contacts =
  //       std::span{data->static_body_contacts,
  //       data->static_body_contact_count};
  //   for (auto const contact : particle_contacts) {
  //     auto const delta = (contact->particles[0] == particle ? 1.0f : -1.0f) *
  //                        dot(position_delta, contact->normal);
  //     contact->separation += delta;
  //   }
  //   for (auto const contact : dynamic_rigid_body_contacts) {
  //     auto const delta = dot(position_delta, contact->normal);
  //     contact->separation += delta;
  //   }
  //   for (auto const contact : static_rigid_body_contacts) {
  //     auto const delta = dot(position_delta, contact->normal);
  //     contact->separation += delta;
  //   }
  // }

  // void update_separations(Rigid_body_handle body,
  //                         Vec3f const &position_delta,
  //                         Vec3f const &orientation_delta) {
  //   auto const data = _intrinsic_state->rigid_bodies->data(body);
  //   auto const particle_contacts =
  //       std::span{data->particle_contacts, data->particle_contact_count};
  //   auto const dynamic_rigid_body_contacts =
  //       std::span{data->rigid_body_contacts, data->rigid_body_contact_count};
  //   auto const static_rigid_body_contacts =
  //       std::span{data->static_body_contacts,
  //       data->static_body_contact_count};
  //   for (auto const contact : particle_contacts) {
  //     auto const delta = -dot(
  //         position_delta + cross(orientation_delta,
  //         contact->relative_position), contact->normal);
  //     contact->separation += delta;
  //   }
  //   for (auto const contact : dynamic_rigid_body_contacts) {
  //     if (contact->bodies[0] == body) {
  //       auto const delta =
  //           dot(position_delta +
  //                   cross(orientation_delta, contact->relative_positions[0]),
  //               contact->normal);
  //       contact->separation += delta;
  //     } else {
  //       auto const delta =
  //           -dot(position_delta +
  //                    cross(orientation_delta,
  //                    contact->relative_positions[1]),
  //                contact->normal);
  //       contact->separation += delta;
  //     }
  //   }
  //   for (auto const contact : static_rigid_body_contacts) {
  //     auto const delta = dot(
  //         position_delta + cross(orientation_delta,
  //         contact->relative_position), contact->normal);
  //     contact->separation += delta;
  //   }
  // }

  // void resolve_contact_velocity(Particle_particle_contact *contact,
  //                               float max_separating_velocity_for_bounce) {
  //   auto const particles = contact->particles;
  //   auto const particle_datas = std::array<Particle_data *, 2>{
  //       _intrinsic_state->particles->data(particles[0]),
  //       _intrinsic_state->particles->data(particles[1]),
  //   };
  //   auto const normal = contact->normal;
  //   auto const separating_velocity = contact->separating_velocity;
  //   auto const velocity_per_impulse =
  //       particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
  //   auto const impulse_per_velocity = 1.0f / velocity_per_impulse;
  //   auto const restitution_coefficient =
  //       separating_velocity < max_separating_velocity_for_bounce
  //           ? 0.5f * (particle_datas[0]->material.restitution_coefficient +
  //                     particle_datas[1]->material.restitution_coefficient)
  //           : 0.0f;
  //   auto const delta_separating_velocity =
  //       -separating_velocity * (1.0f + restitution_coefficient);
  //   auto const separating_impulse_length =
  //       delta_separating_velocity * impulse_per_velocity;
  //   auto const separating_impulse = separating_impulse_length * normal;
  //   auto const frictional_impulse = [&]() {
  //     auto const relative_velocity =
  //         particle_datas[0]->velocity - particle_datas[1]->velocity;
  //     auto const sliding_velocity =
  //         relative_velocity - separating_velocity * normal;
  //     auto const sliding_speed_squared = length_squared(sliding_velocity);
  //     if (sliding_speed_squared != 0.0f) {
  //       auto const sliding_speed = std::sqrt(sliding_speed_squared);
  //       auto const sliding_direction = sliding_velocity / sliding_speed;
  //       auto const frictional_impulse_direction = -sliding_direction;
  //       auto const static_friction_coefficient =
  //           0.5f * (particle_datas[0]->material.static_friction_coefficient +
  //                   particle_datas[1]->material.static_friction_coefficient);
  //       auto const dynamic_friction_coefficient =
  //           0.5f * (particle_datas[0]->material.dynamic_friction_coefficient
  //           +
  //                   particle_datas[1]->material.dynamic_friction_coefficient);
  //       auto const max_static_frictional_impulse_length =
  //           static_friction_coefficient * separating_impulse_length;
  //       auto const max_dynamic_frictional_impulse_length =
  //           dynamic_friction_coefficient * separating_impulse_length;
  //       auto const stopping_impulse_length =
  //           impulse_per_velocity * sliding_speed;
  //       if (stopping_impulse_length <= max_static_frictional_impulse_length)
  //       {
  //         return stopping_impulse_length * frictional_impulse_direction;
  //       } else {
  //         return max_dynamic_frictional_impulse_length *
  //                frictional_impulse_direction;
  //       }
  //     } else {
  //       return Vec3f::zero();
  //     }
  //   }();
  //   auto const impulse = separating_impulse + frictional_impulse;
  //   auto const particle_velocity_deltas =
  //       std::array<Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
  //                            impulse * -particle_datas[1]->inverse_mass};
  //   particle_datas[0]->velocity += particle_velocity_deltas[0];
  //   particle_datas[1]->velocity += particle_velocity_deltas[1];
  //   // for (auto i = 0; i != 2; ++i) {
  //   //   update_separating_velocities(particles[i],
  //   //   particle_velocity_deltas[i]);
  //   // }
  // }

  // void resolve_contact_velocity(Particle_rigid_body_contact *contact,
  //                               float max_separating_velocity_for_bounce) {
  //   auto const particle = contact->particle;
  //   auto const particle_data = _intrinsic_state->particles->data(particle);
  //   auto const body = contact->body;
  //   auto const body_data = _intrinsic_state->rigid_bodies->data(body);
  //   auto const body_relative_contact_position = contact->relative_position;
  //   auto const normal = contact->normal;
  //   auto const separating_velocity = contact->separating_velocity;
  //   auto const body_rotation = Mat3x3f::rotation(body_data->orientation);
  //   auto const body_inverse_rotation = transpose(body_rotation);
  //   auto const body_inverse_inertia_tensor = body_rotation *
  //                                            body_data->inverse_inertia_tensor
  //                                            * body_inverse_rotation;
  //   auto const body_angular_impulse_per_separating_impulse =
  //       cross(body_relative_contact_position, normal);
  //   auto const body_angular_velocity_per_separating_impulse =
  //       body_inverse_inertia_tensor *
  //       body_angular_impulse_per_separating_impulse;
  //   auto const separating_velocity_per_separating_impulse =
  //       particle_data->inverse_mass + body_data->inverse_mass +
  //       dot(cross(body_angular_velocity_per_separating_impulse,
  //                 body_relative_contact_position),
  //           normal);
  //   auto const separating_impulse_per_separating_velocity =
  //       1.0f / separating_velocity_per_separating_impulse;
  //   auto const restitution_coefficient =
  //       separating_velocity < max_separating_velocity_for_bounce
  //           ? 0.5f * (particle_data->material.restitution_coefficient +
  //                     body_data->material.restitution_coefficient)
  //           : 0.0f;
  //   auto const delta_separating_velocity =
  //       -separating_velocity * (1.0f + restitution_coefficient);
  //   auto const separating_impulse_length =
  //       delta_separating_velocity *
  //       separating_impulse_per_separating_velocity;
  //   auto const separating_impulse = separating_impulse_length * normal;
  //   auto const relative_velocity =
  //       particle_data->velocity -
  //       (body_data->velocity +
  //        cross(body_data->angular_velocity, body_relative_contact_position));
  //   auto const sliding_velocity =
  //       relative_velocity - separating_velocity * normal;
  //   auto const sliding_speed_squared = length_squared(sliding_velocity);
  //   auto const frictional_impulse = [&]() {
  //     if (sliding_speed_squared != 0.0f) {
  //       auto const sliding_speed = std::sqrt(sliding_speed_squared);
  //       auto const sliding_direction = sliding_velocity / sliding_speed;
  //       auto const frictional_impulse_direction = -sliding_direction;
  //       auto const body_angular_impulse_per_frictional_impulse =
  //           cross(body_relative_contact_position,
  //           frictional_impulse_direction);
  //       auto const body_angular_velocity_per_frictional_impulse =
  //           body_inverse_inertia_tensor *
  //           body_angular_impulse_per_frictional_impulse;
  //       auto const sliding_velocity_per_frictional_impulse =
  //           particle_data->inverse_mass + body_data->inverse_mass +
  //           dot(cross(body_angular_velocity_per_frictional_impulse,
  //                     body_relative_contact_position),
  //               frictional_impulse_direction);
  //       auto const frictional_impulse_per_sliding_velocity =
  //           1.0f / sliding_velocity_per_frictional_impulse;
  //       auto const static_friction_coefficient =
  //           0.5f * (particle_data->material.static_friction_coefficient +
  //                   body_data->material.static_friction_coefficient);
  //       auto const dynamic_friction_coefficient =
  //           0.5f * (particle_data->material.dynamic_friction_coefficient +
  //                   body_data->material.dynamic_friction_coefficient);
  //       auto const max_static_friction_impulse_length =
  //           static_friction_coefficient * separating_impulse_length;
  //       auto const max_dynamic_friction_impulse_length =
  //           dynamic_friction_coefficient * separating_impulse_length;
  //       auto const stopping_impulse_length =
  //           frictional_impulse_per_sliding_velocity * sliding_speed;
  //       if (stopping_impulse_length <= max_static_friction_impulse_length) {
  //         return stopping_impulse_length * frictional_impulse_direction;
  //       } else {
  //         return max_dynamic_friction_impulse_length *
  //                frictional_impulse_direction;
  //       }
  //     } else {
  //       return Vec3f::zero();
  //     }
  //   }();
  //   auto const impulse = separating_impulse + frictional_impulse;
  //   auto const particle_velocity_delta = impulse *
  //   particle_data->inverse_mass; auto const body_velocity_delta = -impulse *
  //   body_data->inverse_mass; auto const body_angular_velocity_delta =
  //       body_inverse_inertia_tensor *
  //       cross(body_relative_contact_position, -impulse);
  //   particle_data->velocity += particle_velocity_delta;
  //   body_data->velocity += body_velocity_delta;
  //   body_data->angular_velocity += body_angular_velocity_delta;
  //   // update_separating_velocities(particle, particle_velocity_delta);
  //   // update_separating_velocities(
  //   //     body, body_velocity_delta, body_angular_velocity_delta);
  // }

  // void resolve_contact_velocity(Particle_static_body_contact *contact,
  //                               Vec3f const &gravitational_velocity_delta,
  //                               float max_separating_velocity_for_bounce) {
  //   auto const particle = contact->particle;
  //   auto const particle_data = _intrinsic_state->particles->data(particle);
  //   auto const body = contact->body;
  //   auto const body_data = _intrinsic_state->static_bodies->data(body);
  //   auto const normal = contact->normal;
  //   auto const separating_velocity = contact->separating_velocity;
  //   auto const restitution_coefficient =
  //       separating_velocity < max_separating_velocity_for_bounce
  //           ? 0.5f * (particle_data->material.restitution_coefficient +
  //                     body_data->material.restitution_coefficient)
  //           : 0.0f;
  //   auto const delta_separating_velocity =
  //       -separating_velocity -
  //       std::min(separating_velocity -
  //                    dot(gravitational_velocity_delta, normal),
  //                0.0f) *
  //           restitution_coefficient;
  //   auto const relative_velocity = particle_data->velocity;
  //   auto const sliding_velocity =
  //       relative_velocity - separating_velocity * normal;
  //   auto const sliding_velocity_length_squared =
  //       length_squared(sliding_velocity);
  //   auto const static_friction_coefficient =
  //       0.5f * (particle_data->material.static_friction_coefficient +
  //               body_data->material.static_friction_coefficient);
  //   auto const dynamic_friction_coefficient =
  //       0.5f * (particle_data->material.dynamic_friction_coefficient +
  //               body_data->material.dynamic_friction_coefficient);
  //   auto const max_static_friction_delta_sliding_velocity =
  //       static_friction_coefficient * delta_separating_velocity;
  //   auto const max_static_friction_delta_sliding_velocity_squared =
  //       max_static_friction_delta_sliding_velocity *
  //       max_static_friction_delta_sliding_velocity;
  //   auto const max_dynamic_friction_delta_sliding_velocity =
  //       dynamic_friction_coefficient * delta_separating_velocity;
  //   auto const delta_sliding_velocity =
  //       sliding_velocity_length_squared != 0.0f
  //           ? (sliding_velocity_length_squared <=
  //                      max_static_friction_delta_sliding_velocity_squared
  //                  ? -sliding_velocity
  //                  : -sliding_velocity *
  //                        max_dynamic_friction_delta_sliding_velocity /
  //                        std::sqrt(sliding_velocity_length_squared))
  //           : Vec3f::zero();
  //   auto const particle_velocity_delta =
  //       delta_separating_velocity * normal + delta_sliding_velocity;
  //   particle_data->velocity += particle_velocity_delta;
  //   // update_separating_velocities(particle, particle_velocity_delta);
  // }

  // void resolve_contact_velocity(Rigid_body_rigid_body_contact *contact,
  //                               float max_separating_velocity_for_bounce) {
  //   auto const bodies = contact->bodies;
  //   auto const datas = std::array<Rigid_body_data *, 2>{
  //       _intrinsic_state->rigid_bodies->data(bodies[0]),
  //       _intrinsic_state->rigid_bodies->data(bodies[1]),
  //   };
  //   auto const relative_positions = contact->relative_positions;
  //   auto const normal = contact->normal;
  //   auto const separating_velocity = contact->separating_velocity;
  //   auto const rotations = std::array<Mat3x3f, 2>{
  //       Mat3x3f::rotation(datas[0]->orientation),
  //       Mat3x3f::rotation(datas[1]->orientation),
  //   };
  //   auto const inverse_rotations = std::array<Mat3x3f, 2>{
  //       transpose(rotations[0]),
  //       transpose(rotations[1]),
  //   };
  //   auto const inverse_inertia_tensors = std::array<Mat3x3f, 2>{
  //       rotations[0] * datas[0]->inverse_inertia_tensor *
  //       inverse_rotations[0], rotations[1] * datas[1]->inverse_inertia_tensor
  //       * inverse_rotations[1],
  //   };
  //   auto const angular_impulses_per_separating_impulse = std::array<Vec3f,
  //   2>{
  //       cross(relative_positions[0], normal),
  //       cross(relative_positions[1], normal),
  //   };
  //   auto const angular_velocities_per_separating_impulse = std::array<Vec3f,
  //   2>{
  //       inverse_inertia_tensors[0] *
  //       angular_impulses_per_separating_impulse[0],
  //       inverse_inertia_tensors[1] *
  //       angular_impulses_per_separating_impulse[1],
  //   };
  //   auto const separating_velocity_per_separating_impulse =
  //       datas[0]->inverse_mass + datas[1]->inverse_mass +
  //       dot(cross(angular_velocities_per_separating_impulse[0],
  //                 relative_positions[0]) +
  //               cross(angular_velocities_per_separating_impulse[1],
  //                     relative_positions[1]),
  //           normal);
  //   auto const separating_impulse_per_separating_velocity =
  //       1.0f / separating_velocity_per_separating_impulse;
  //   auto const restitution_coefficient =
  //       separating_velocity < max_separating_velocity_for_bounce
  //           ? 0.5f * (datas[0]->material.restitution_coefficient +
  //                     datas[1]->material.restitution_coefficient)
  //           : 0.0f;
  //   auto const delta_separating_velocity =
  //       -separating_velocity * (1.0f + restitution_coefficient);
  //   auto const separating_impulse_length =
  //       delta_separating_velocity *
  //       separating_impulse_per_separating_velocity;
  //   auto const separating_impulse = separating_impulse_length * normal;
  //   auto const frictional_impulse = [&]() {
  //     auto const relative_velocity =
  //         (datas[0]->velocity +
  //          cross(datas[0]->angular_velocity, relative_positions[0])) -
  //         (datas[1]->velocity +
  //          cross(datas[1]->angular_velocity, relative_positions[1]));
  //     auto const sliding_velocity =
  //         relative_velocity - separating_velocity * normal;
  //     auto const sliding_speed_squared = length_squared(sliding_velocity);
  //     if (sliding_speed_squared != 0.0f) {
  //       auto const sliding_speed = std::sqrt(sliding_speed_squared);
  //       auto const sliding_direction = sliding_velocity / sliding_speed;
  //       auto const frictional_impulse_direction = -sliding_direction;
  //       auto const angular_impulses_per_frictional_impulse =
  //           std::array<Vec3f, 2>{
  //               cross(relative_positions[0], frictional_impulse_direction),
  //               cross(relative_positions[1], frictional_impulse_direction),
  //           };
  //       auto const angular_velocities_per_frictional_impulse =
  //           std::array<Vec3f, 2>{
  //               inverse_inertia_tensors[0] *
  //                   angular_impulses_per_frictional_impulse[0],
  //               inverse_inertia_tensors[1] *
  //                   angular_impulses_per_frictional_impulse[1],
  //           };
  //       auto const sliding_velocity_per_frictional_impulse =
  //           datas[0]->inverse_mass + datas[1]->inverse_mass +
  //           dot(cross(angular_velocities_per_frictional_impulse[0],
  //                     relative_positions[0]),
  //               cross(angular_velocities_per_frictional_impulse[1],
  //                     relative_positions[1]));
  //       auto const frictional_impulse_per_sliding_velocity =
  //           1.0f / sliding_velocity_per_frictional_impulse;
  //       auto const static_friction_coefficient =
  //           0.5f * (datas[0]->material.static_friction_coefficient +
  //                   datas[1]->material.static_friction_coefficient);
  //       auto const dynamic_friction_coefficient =
  //           0.5f * (datas[0]->material.dynamic_friction_coefficient +
  //                   datas[1]->material.dynamic_friction_coefficient);
  //       auto const max_static_friction_impulse_length =
  //           static_friction_coefficient * separating_impulse_length;
  //       auto const max_dynamic_friction_impulse_length =
  //           dynamic_friction_coefficient * separating_impulse_length;
  //       auto const stopping_impulse_length =
  //           frictional_impulse_per_sliding_velocity * sliding_speed;
  //       if (stopping_impulse_length <= max_static_friction_impulse_length) {
  //         return stopping_impulse_length * frictional_impulse_direction;
  //       } else {
  //         return max_dynamic_friction_impulse_length *
  //                frictional_impulse_direction;
  //       }
  //     } else {
  //       return Vec3f::zero();
  //     }
  //   }();
  //   auto const impulse = separating_impulse + frictional_impulse;
  //   auto const body_velocity_deltas = std::array<Vec3f, 2>{
  //       impulse * datas[0]->inverse_mass,
  //       -impulse * datas[1]->inverse_mass,
  //   };
  //   auto const body_angular_velocity_deltas = std::array<Vec3f, 2>{
  //       inverse_inertia_tensors[0] * cross(relative_positions[0], impulse),
  //       inverse_inertia_tensors[1] * cross(relative_positions[1], -impulse),
  //   };
  //   for (auto i = 0; i != 2; ++i) {
  //     datas[i]->velocity += body_velocity_deltas[i];
  //     datas[i]->angular_velocity += body_angular_velocity_deltas[i];
  //     // update_separating_velocities(
  //     //     bodies[i], body_velocity_deltas[i],
  //     //     body_angular_velocity_deltas[i]);
  //   }
  // }

  // void resolve_contact_velocity(Rigid_body_static_body_contact *contact,
  //                               Vec3f const &gravitational_velocity_delta,
  //                               float max_separating_velocity_for_bounce) {
  //   auto const dynamic_body = contact->rigid_body;
  //   auto const dynamic_body_data =
  //       _intrinsic_state->rigid_bodies->data(dynamic_body);
  //   auto const static_body = contact->static_body;
  //   auto const static_body_data =
  //       _intrinsic_state->static_bodies->data(static_body);
  //   auto const dynamic_body_relative_contact_position =
  //       contact->relative_position;
  //   auto const normal = contact->normal;
  //   auto const separating_velocity = contact->separating_velocity;
  //   auto const dynamic_body_rotation =
  //       Mat3x3f::rotation(dynamic_body_data->orientation);
  //   auto const dynamic_body_inverse_rotation =
  //   transpose(dynamic_body_rotation); auto const
  //   dynamic_body_inverse_inertia_tensor =
  //       dynamic_body_rotation * dynamic_body_data->inverse_inertia_tensor *
  //       dynamic_body_inverse_rotation;
  //   auto const angular_impulse_per_separating_impulse =
  //       cross(dynamic_body_relative_contact_position, normal);
  //   auto const angular_velocity_per_separating_impulse =
  //       dynamic_body_inverse_inertia_tensor *
  //       angular_impulse_per_separating_impulse;
  //   auto const separating_velocity_per_separating_impulse =
  //       dynamic_body_data->inverse_mass +
  //       dot(cross(angular_velocity_per_separating_impulse,
  //                 dynamic_body_relative_contact_position),
  //           normal);
  //   auto const separating_impulse_per_separating_velocity =
  //       1.0f / separating_velocity_per_separating_impulse;
  //   auto const restitution_coefficient =
  //       separating_velocity < max_separating_velocity_for_bounce
  //           ? 0.5f * (static_body_data->material.restitution_coefficient +
  //                     dynamic_body_data->material.restitution_coefficient)
  //           : 0.0f;
  //   auto const delta_separating_velocity =
  //       -separating_velocity -
  //       std::min(separating_velocity -
  //                    dot(gravitational_velocity_delta, normal),
  //                0.0f) *
  //           restitution_coefficient;
  //   auto const separating_impulse_length =
  //       delta_separating_velocity *
  //       separating_impulse_per_separating_velocity;
  //   auto const separating_impulse = separating_impulse_length * normal;
  //   auto const frictional_impulse = [&]() {
  //     auto const relative_velocity =
  //         dynamic_body_data->velocity +
  //         cross(dynamic_body_data->angular_velocity,
  //               dynamic_body_relative_contact_position);
  //     auto const sliding_velocity =
  //         relative_velocity - separating_velocity * normal;
  //     auto const sliding_speed_squared = length_squared(sliding_velocity);
  //     if (sliding_speed_squared != 0.0f) {
  //       auto const sliding_speed = std::sqrt(sliding_speed_squared);
  //       auto const sliding_direction = sliding_velocity / sliding_speed;
  //       auto const frictional_impulse_direction = -sliding_direction;
  //       auto const angular_impulse_per_frictional_impulse =
  //           cross(dynamic_body_relative_contact_position,
  //                 frictional_impulse_direction);
  //       auto const angular_velocity_per_frictional_impulse =
  //           dynamic_body_inverse_inertia_tensor *
  //           angular_impulse_per_frictional_impulse;
  //       auto const sliding_velocity_per_frictional_impulse =
  //           dynamic_body_data->inverse_mass +
  //           dot(cross(angular_velocity_per_frictional_impulse,
  //                     dynamic_body_relative_contact_position),
  //               frictional_impulse_direction);
  //       auto const frictional_impulse_per_sliding_velocity =
  //           1.0f / sliding_velocity_per_frictional_impulse;
  //       auto const static_friction_coefficient =
  //           0.5f * (dynamic_body_data->material.static_friction_coefficient +
  //                   static_body_data->material.static_friction_coefficient);
  //       auto const dynamic_friction_coefficient =
  //           0.5f * (dynamic_body_data->material.dynamic_friction_coefficient
  //           +
  //                   static_body_data->material.dynamic_friction_coefficient);
  //       auto const max_static_friction_impulse_length =
  //           static_friction_coefficient * separating_impulse_length;
  //       auto const max_dynamic_friction_impulse_length =
  //           dynamic_friction_coefficient * separating_impulse_length;
  //       auto const stopping_impulse_length =
  //           frictional_impulse_per_sliding_velocity * sliding_speed;
  //       if (stopping_impulse_length <= max_static_friction_impulse_length) {
  //         return stopping_impulse_length * frictional_impulse_direction;
  //       } else {
  //         return max_dynamic_friction_impulse_length *
  //                frictional_impulse_direction;
  //       }
  //     } else {
  //       return Vec3f::zero();
  //     }
  //   }();
  //   auto const impulse = separating_impulse + frictional_impulse;
  //   auto const dynamic_body_velocity_delta =
  //       dynamic_body_data->inverse_mass * impulse;
  //   auto const dynamic_body_angular_velocity_delta =
  //       dynamic_body_inverse_inertia_tensor *
  //       cross(dynamic_body_relative_contact_position, impulse);
  //   dynamic_body_data->velocity += dynamic_body_velocity_delta;
  //   dynamic_body_data->angular_velocity +=
  //   dynamic_body_angular_velocity_delta;
  //   // update_separating_velocities(dynamic_body,
  //   //                              dynamic_body_velocity_delta,
  //   //                              dynamic_body_angular_velocity_delta);
  // }

  // void update_separating_velocities(Particle_handle particle,
  //                                   Vec3f const &velocity_delta) {
  //   auto const data = _intrinsic_state->particles->data(particle);
  //   auto const particle_contacts =
  //       std::span{data->particle_contacts, data->particle_contact_count};
  //   auto const dynamic_rigid_body_contacts =
  //       std::span{data->rigid_body_contacts, data->rigid_body_contact_count};
  //   auto const static_rigid_body_contacts =
  //       std::span{data->static_body_contacts,
  //       data->static_body_contact_count};
  //   for (auto const contact : particle_contacts) {
  //     auto const delta = (contact->particles[0] == particle ? 1.0f : -1.0f) *
  //                        dot(velocity_delta, contact->normal);
  //     contact->separating_velocity += delta;
  //   }
  //   for (auto const contact : dynamic_rigid_body_contacts) {
  //     auto const delta = dot(velocity_delta, contact->normal);
  //     contact->separating_velocity += delta;
  //   }
  //   for (auto const contact : static_rigid_body_contacts) {
  //     auto const delta = dot(velocity_delta, contact->normal);
  //     contact->separating_velocity += delta;
  //   }
  // }

  // void update_separating_velocities(Rigid_body_handle body,
  //                                   Vec3f const &velocity_delta,
  //                                   Vec3f const &angular_velocity_delta) {
  //   auto const data = _intrinsic_state->rigid_bodies->data(body);
  //   auto const particle_contacts =
  //       std::span{data->particle_contacts, data->particle_contact_count};
  //   auto const rigid_body_contacts =
  //       std::span{data->rigid_body_contacts, data->rigid_body_contact_count};
  //   auto const static_body_contacts =
  //       std::span{data->static_body_contacts,
  //       data->static_body_contact_count};
  //   for (auto const contact : particle_contacts) {
  //     auto const delta =
  //         -dot(velocity_delta +
  //                  cross(angular_velocity_delta, contact->relative_position),
  //              contact->normal);
  //     contact->separating_velocity += delta;
  //   }
  //   for (auto const contact : rigid_body_contacts) {
  //     if (contact->bodies[0] == body) {
  //       auto const delta =
  //           dot(velocity_delta + cross(angular_velocity_delta,
  //                                      contact->relative_positions[0]),
  //               contact->normal);
  //       contact->separating_velocity += delta;
  //     } else {
  //       auto const delta =
  //           -dot(velocity_delta + cross(angular_velocity_delta,
  //                                       contact->relative_positions[1]),
  //                contact->normal);
  //       contact->separating_velocity += delta;
  //     }
  //   }
  //   for (auto const contact : static_body_contacts) {
  //     auto const delta = dot(velocity_delta + cross(angular_velocity_delta,
  //                                                   contact->relative_position),
  //                            contact->normal);
  //     contact->separating_velocity += delta;
  //   }
  // }

  Intrinsic_state const *_intrinsic_state;
  // std::size_t _group_index;
};

class Contact_group_storage {
  using Allocator = Stack_allocator<>;

public:
  static constexpr std::size_t memory_requirement(std::size_t max_contact_count,
                                                  std::size_t max_group_count) {
    return Allocator::memory_requirement(
        {decltype(_contacts)::memory_requirement(max_contact_count),
         decltype(_groups)::memory_requirement(max_group_count)});
  }

  constexpr Contact_group_storage() = default;

  explicit Contact_group_storage(Block block,
                                 std::size_t max_contact_count,
                                 std::size_t max_group_count)
      : Contact_group_storage{block.begin, max_contact_count, max_group_count} {
  }

  explicit Contact_group_storage(void *block,
                                 std::size_t max_contact_count,
                                 std::size_t max_group_count) {
    auto allocator = Allocator{make_block(
        block, memory_requirement(max_contact_count, max_group_count))};
    _contacts = make_contact_list(allocator, max_contact_count).second;
    _groups = util::make_list<Group>(allocator, max_group_count).second;
  }

  std::size_t contact_count() const noexcept { return _contacts.size(); }

  std::variant<Particle_particle_contact *,
               Particle_rigid_body_contact *,
               Particle_static_body_contact *,
               Rigid_body_rigid_body_contact *,
               Rigid_body_static_body_contact *>
  contact(std::size_t object_index) const noexcept {
    return _contacts.at(object_index);
  }

  std::size_t group_count() const noexcept { return _groups.size(); }

  std::size_t group_begin(std::size_t group_index) const noexcept {
    return _groups[group_index].begin;
  }

  std::size_t group_end(std::size_t group_index) const noexcept {
    return _groups[group_index].end;
  }

  // Contact_group_solve_task const *
  // group_solve_task(std::size_t group_index) const noexcept {
  //   return &_groups[group_index].solve_task;
  // }

  // Contact_group_solve_task *group_solve_task(std::size_t group_index)
  // noexcept {
  //   return &_groups[group_index].solve_task;
  // }

  void clear() noexcept {
    _contacts.clear();
    _groups.clear();
  }

  void begin_group() {
    auto const index = static_cast<std::uint32_t>(_contacts.size());
    _groups.push_back({index, index /*, {nullptr, _groups.size()}*/});
  }

  void add_to_group(Particle_particle_contact *contact) {
    _contacts.push_back(contact);
    ++_groups.back().end;
  }

  void add_to_group(Particle_rigid_body_contact *contact) {
    _contacts.push_back(contact);
    ++_groups.back().end;
  }

  void add_to_group(Particle_static_body_contact *contact) {
    _contacts.push_back(contact);
    ++_groups.back().end;
  }

  void add_to_group(Rigid_body_rigid_body_contact *contact) {
    _contacts.push_back(contact);
    ++_groups.back().end;
  }

  void add_to_group(Rigid_body_static_body_contact *contact) {
    _contacts.push_back(contact);
    ++_groups.back().end;
  }

  void add_to_group(std::variant<Particle_particle_contact *,
                                 Particle_rigid_body_contact *,
                                 Particle_static_body_contact *,
                                 Rigid_body_rigid_body_contact *,
                                 Rigid_body_static_body_contact *> contact) {
    std::visit([this](auto &&arg) { add_to_group(arg); }, contact);
  }

  // std::optional<std::variant<Particle_particle_contact *,
  //                            Particle_rigid_body_contact *,
  //                            Particle_static_body_contact *,
  //                            Rigid_body_rigid_body_contact *,
  //                            Rigid_body_static_body_contact *>>
  // select_contact_position_solve(std::size_t group_index,
  //                               float max_separation) const noexcept {
  //   return _contacts.select_contact_position_solve(
  //       max_separation, group_begin(group_index), group_end(group_index));
  // }

  // std::optional<std::variant<Particle_particle_contact *,
  //                            Particle_rigid_body_contact *,
  //                            Particle_static_body_contact *,
  //                            Rigid_body_rigid_body_contact *,
  //                            Rigid_body_static_body_contact *>>
  // select_contact_velocity_solve(std::size_t group_index,
  //                               float max_separating_velocity) const noexcept
  //                               {
  //   return _contacts.select_contact_velocity_solve(max_separating_velocity,
  //                                                  group_begin(group_index),
  //                                                  group_end(group_index));
  // }

private:
  struct Group {
    std::uint32_t begin;
    std::uint32_t end;
    // Contact_group_solve_task solve_task;
  };

  Contact_list _contacts;
  List<Group> _groups;
};

template <typename Allocator>
std::pair<Block, Contact_group_storage>
make_contact_group_storage(Allocator &allocator,
                           std::size_t max_contact_count,
                           std::size_t max_group_count) {
  auto const block = allocator.alloc(Contact_group_storage::memory_requirement(
      max_contact_count, max_group_count));
  return {block,
          Contact_group_storage{block, max_contact_count, max_group_count}};
}

// void Contact_group_solve_task::run(unsigned) {
//   auto const &_contact_groups = *_intrinsic_state->contact_groups;
//   auto const min_desired_position_iterations_per_contact =
//       _intrinsic_state->min_desired_position_iterations_per_contact;
//   auto const max_desired_position_iterations_per_contact =
//       _intrinsic_state->max_desired_position_iterations_per_contact;
//   auto const max_position_iterations =
//       _intrinsic_state->max_position_iterations;
//   auto const min_desired_velocity_iterations_per_contact =
//       _intrinsic_state->min_desired_velocity_iterations_per_contact;
//   auto const max_desired_velocity_iterations_per_contact =
//       _intrinsic_state->max_desired_velocity_iterations_per_contact;
//   auto const max_velocity_iterations =
//       _intrinsic_state->max_velocity_iterations;
//   auto const early_out_contact_separation =
//       _intrinsic_state->early_out_contact_separation;
//   auto const early_out_contact_separating_velocity =
//       _intrinsic_state->early_out_contact_separating_velocity;
//   auto const gravitational_delta_velocity =
//       _intrinsic_state->gravitational_delta_velocity;
//   auto const restitution_max_separating_velocity =
//       _intrinsic_state->restitution_max_separating_velocity;
//   auto position_iterations = 0;
//   auto const group_begin = _contact_groups.group_begin(_group_index);
//   auto const group_end = _contact_groups.group_end(_group_index);
//   auto const group_size = group_end - group_begin;
//   auto const min_desired_position_iterations =
//       min_desired_position_iterations_per_contact * group_size;
//   for (auto i = std::size_t{}; position_iterations < max_position_iterations
//   &&
//                                i != min_desired_position_iterations;
//        ++i) {
//     if (auto const contact =
//             _contact_groups.select_contact_position_solve(_group_index,
//             0.0f)) {
//       std::visit([this](auto &&arg) { resolve_contact_position(arg); },
//                  *contact);
//       ++position_iterations;
//     } else {
//       break;
//     }
//   }
//   auto const optional_desired_position_iterations =
//       (max_desired_position_iterations_per_contact -
//        min_desired_position_iterations_per_contact) *
//       group_size;
//   for (auto i = std::size_t{}; position_iterations < max_position_iterations
//   &&
//                                i != optional_desired_position_iterations;
//        ++i) {
//     if (auto const contact = _contact_groups.select_contact_position_solve(
//             _group_index, early_out_contact_separation)) {
//       std::visit([this](auto &&arg) { resolve_contact_position(arg); },
//                  *contact);
//       ++position_iterations;
//     } else {
//       break;
//     }
//   }
//   auto velocity_iterations = 0;
//   auto const min_desired_velocity_iterations =
//       min_desired_velocity_iterations_per_contact * group_size;
//   for (auto i = std::size_t{}; velocity_iterations < max_velocity_iterations
//   &&
//                                i != min_desired_velocity_iterations;
//        ++i) {
//     if (auto const contact =
//             _contact_groups.select_contact_velocity_solve(_group_index,
//             0.0f)) {
//       std::visit(
//           [=, this](auto &&arg) {
//             using T = std::decay_t<decltype(arg)>;
//             if constexpr (std::is_same_v<T, Particle_static_body_contact *>
//             ||
//                           std::is_same_v<T, Rigid_body_static_body_contact
//                           *>) {
//               resolve_contact_velocity(arg,
//                                        gravitational_delta_velocity,
//                                        restitution_max_separating_velocity);
//             } else {
//               resolve_contact_velocity(arg,
//                                        restitution_max_separating_velocity);
//             }
//           },
//           *contact);
//       ++velocity_iterations;
//     } else {
//       break;
//     }
//   }
//   auto const optional_desired_velocity_iterations =
//       (max_desired_velocity_iterations_per_contact -
//        min_desired_velocity_iterations_per_contact) *
//       group_size;
//   for (auto i = std::size_t{}; velocity_iterations < max_velocity_iterations
//   &&
//                                i != optional_desired_velocity_iterations;
//        ++i) {
//     if (auto const contact = _contact_groups.select_contact_velocity_solve(
//             _group_index, early_out_contact_separating_velocity)) {
//       std::visit(
//           [=, this](auto &&arg) {
//             using T = std::decay_t<decltype(arg)>;
//             if constexpr (std::is_same_v<T, Particle_static_body_contact *>
//             ||
//                           std::is_same_v<T, Rigid_body_static_body_contact
//                           *>) {
//               resolve_contact_velocity(arg,
//                                        gravitational_delta_velocity,
//                                        restitution_max_separating_velocity);
//             } else {
//               resolve_contact_velocity(arg,
//                                        restitution_max_separating_velocity);
//             }
//           },
//           *contact);
//       ++velocity_iterations;
//     } else {
//       break;
//     }
//   }
//   _intrinsic_state->latch->count_down();
// }
} // namespace

class World::Impl {
public:
  friend class World;

  static constexpr std::size_t
  memory_requirement(World_create_info const &create_info) {
    return Stack_allocator<>::memory_requirement({
        decltype(_aabb_tree)::memory_requirement(
            create_info.max_aabb_tree_leaf_nodes,
            create_info.max_aabb_tree_internal_nodes),
        decltype(_particle_particle_neighbor_pairs)::memory_requirement(
            create_info.max_particle_particle_neighbor_pairs),
        decltype(_particle_rigid_body_neighbor_pairs)::memory_requirement(
            create_info.max_particle_rigid_body_neighbor_pairs),
        decltype(_particle_static_body_neighbor_pairs)::memory_requirement(
            create_info.max_particle_static_body_neighbor_pairs),
        decltype(_rigid_body_rigid_body_neighbor_pairs)::memory_requirement(
            create_info.max_rigid_body_rigid_body_neighbor_pairs),
        decltype(_rigid_body_static_body_neighbor_pairs)::memory_requirement(
            create_info.max_rigid_body_static_body_neighbor_pairs),
        decltype(_particle_neighbors)::memory_requirement(
            2 * create_info.max_particle_particle_neighbor_pairs +
            create_info.max_particle_rigid_body_neighbor_pairs),
        decltype(_rigid_body_neighbors)::memory_requirement(
            create_info.max_particle_rigid_body_neighbor_pairs +
            2 * create_info.max_rigid_body_rigid_body_neighbor_pairs),
        decltype(_static_body_neighbors)::memory_requirement(
            create_info.max_particle_static_body_neighbor_pairs +
            create_info.max_rigid_body_static_body_neighbor_pairs),
        decltype(_neighbor_groups)::memory_requirement(
            create_info.max_particles + create_info.max_rigid_bodies,
            create_info.max_contact_groups),
        decltype(_awake_neighbor_groups)::memory_requirement(
            create_info.max_contact_groups),
        decltype(_particle_particle_contacts)::memory_requirement(
            create_info.max_particle_particle_neighbor_pairs),
        decltype(_particle_rigid_body_contacts)::memory_requirement(
            create_info.max_particle_rigid_body_neighbor_pairs),
        decltype(_particle_static_body_contacts)::memory_requirement(
            create_info.max_particle_static_body_neighbor_pairs),
        decltype(_rigid_body_rigid_body_contacts)::memory_requirement(
            create_info.max_rigid_body_rigid_body_neighbor_pairs),
        decltype(_rigid_body_static_body_contacts)::memory_requirement(
            create_info.max_rigid_body_static_body_neighbor_pairs),
        // decltype(_rigid_body_rigid_body_contact_pairs)::memory_requirement(
        //     create_info.max_rigid_body_rigid_body_neighbor_pairs),
        // decltype(_rigid_body_static_body_contact_pairs)::memory_requirement(
        //     create_info.max_rigid_body_static_body_neighbor_pairs),
        decltype(_particle_particle_contact_ptrs)::memory_requirement(
            2 * create_info.max_particle_particle_neighbor_pairs),
        decltype(_particle_rigid_body_contact_ptrs)::memory_requirement(
            2 * create_info.max_particle_rigid_body_neighbor_pairs),
        decltype(_particle_static_body_contact_ptrs)::memory_requirement(
            create_info.max_particle_static_body_neighbor_pairs),
        decltype(_rigid_body_rigid_body_contact_ptrs)::memory_requirement(
            2 * create_info.max_rigid_body_rigid_body_neighbor_pairs),
        decltype(_rigid_body_static_body_contact_ptrs)::memory_requirement(
            create_info.max_rigid_body_static_body_neighbor_pairs),
        // decltype(_contact_cache)::memory_requirement(
        //     create_info.max_rigid_body_rigid_body_neighbor_pairs,
        //     create_info.max_rigid_body_static_body_neighbor_pairs),
        // decltype(_contact_groups)::memory_requirement(
        //     create_info.max_particle_particle_neighbor_pairs +
        //         create_info.max_particle_rigid_body_neighbor_pairs +
        //         create_info.max_particle_static_body_neighbor_pairs +
        //         create_info.max_rigid_body_rigid_body_neighbor_pairs +
        //         create_info.max_rigid_body_static_body_neighbor_pairs,
        //     create_info.max_contact_groups),
        // decltype(_contact_group_fringe)::memory_requirement(
        //     create_info.max_contact_group_fringe_size),
    });
  }

  explicit Impl(World_create_info const &create_info)
      : _particles{create_info.max_particles},
        _static_bodies{create_info.max_static_bodies},
        _rigid_bodies{create_info.max_rigid_bodies},
        _gravitational_acceleration{create_info.gravitational_acceleration} {
    _block = util::System_allocator::instance()->alloc(
        memory_requirement(create_info));
    auto allocator = Stack_allocator<>{_block};
    _aabb_tree = make_aabb_tree<Aabb_tree_payload_t>(
                     allocator,
                     create_info.max_aabb_tree_leaf_nodes,
                     create_info.max_aabb_tree_internal_nodes)
                     .second;
    _particle_particle_neighbor_pairs =
        util::make_list<std::pair<Particle_handle, Particle_handle>>(
            allocator, create_info.max_particle_particle_neighbor_pairs)
            .second;
    _particle_rigid_body_neighbor_pairs =
        util::make_list<std::pair<Particle_handle, Rigid_body_handle>>(
            allocator, create_info.max_particle_rigid_body_neighbor_pairs)
            .second;
    _particle_static_body_neighbor_pairs =
        util::make_list<std::pair<Particle_handle, Static_body_handle>>(
            allocator, create_info.max_particle_static_body_neighbor_pairs)
            .second;
    _rigid_body_rigid_body_neighbor_pairs =
        util::make_list<std::pair<Rigid_body_handle, Rigid_body_handle>>(
            allocator, create_info.max_rigid_body_rigid_body_neighbor_pairs)
            .second;
    _rigid_body_static_body_neighbor_pairs =
        util::make_list<std::pair<Rigid_body_handle, Static_body_handle>>(
            allocator, create_info.max_rigid_body_static_body_neighbor_pairs)
            .second;
    _particle_neighbors =
        util::make_list<Particle_handle>(
            allocator,
            2 * create_info.max_particle_particle_neighbor_pairs +
                create_info.max_particle_rigid_body_neighbor_pairs)
            .second;
    _rigid_body_neighbors =
        util::make_list<Rigid_body_handle>(
            allocator,
            create_info.max_particle_rigid_body_neighbor_pairs +
                2 * create_info.max_rigid_body_rigid_body_neighbor_pairs)
            .second;
    _static_body_neighbors =
        util::make_list<Static_body_handle>(
            allocator,
            create_info.max_particle_static_body_neighbor_pairs +
                create_info.max_rigid_body_static_body_neighbor_pairs)
            .second;
    _neighbor_groups =
        make_neighbor_group_storage(allocator,
                                    create_info.max_particles +
                                        create_info.max_rigid_bodies,
                                    create_info.max_contact_groups)
            .second;
    _awake_neighbor_groups =
        util::make_list<std::size_t>(allocator, create_info.max_contact_groups)
            .second;
    _particle_particle_contacts =
        util::make_list<Particle_particle_contact>(
            allocator, create_info.max_particle_particle_neighbor_pairs)
            .second;
    _particle_rigid_body_contacts =
        util::make_list<Particle_rigid_body_contact>(
            allocator, create_info.max_particle_rigid_body_neighbor_pairs)
            .second;
    _particle_static_body_contacts =
        util::make_list<Particle_static_body_contact>(
            allocator, create_info.max_particle_static_body_neighbor_pairs)
            .second;
    _rigid_body_rigid_body_contacts =
        util::make_list<Rigid_body_rigid_body_contact>(
            allocator, create_info.max_rigid_body_rigid_body_neighbor_pairs)
            .second;
    _rigid_body_static_body_contacts =
        util::make_list<Rigid_body_static_body_contact>(
            allocator, create_info.max_rigid_body_static_body_neighbor_pairs)
            .second;
    // _rigid_body_rigid_body_contact_pairs =
    //     util::make_list<std::pair<Rigid_body_handle, Rigid_body_handle>>(
    //         allocator, create_info.max_rigid_body_rigid_body_neighbor_pairs)
    //         .second;
    // _rigid_body_static_body_contact_pairs =
    //     util::make_list<std::pair<Rigid_body_handle, Static_body_handle>>(
    //         allocator, create_info.max_rigid_body_static_body_neighbor_pairs)
    //         .second;
    _particle_particle_contact_ptrs =
        util::make_list<Particle_particle_contact *>(
            allocator, 2 * create_info.max_particle_particle_neighbor_pairs)
            .second;
    _particle_rigid_body_contact_ptrs =
        util::make_list<Particle_rigid_body_contact *>(
            allocator, 2 * create_info.max_particle_rigid_body_neighbor_pairs)
            .second;
    _particle_static_body_contact_ptrs =
        util::make_list<Particle_static_body_contact *>(
            allocator, create_info.max_particle_static_body_neighbor_pairs)
            .second;
    _rigid_body_rigid_body_contact_ptrs =
        util::make_list<Rigid_body_rigid_body_contact *>(
            allocator, 2 * create_info.max_rigid_body_rigid_body_neighbor_pairs)
            .second;
    _rigid_body_static_body_contact_ptrs =
        util::make_list<Rigid_body_static_body_contact *>(
            allocator, create_info.max_rigid_body_static_body_neighbor_pairs)
            .second;
    // _contact_cache = make_contact_cache(
    //                      allocator,
    //                      create_info.max_rigid_body_rigid_body_neighbor_pairs,
    //                      create_info.max_rigid_body_static_body_neighbor_pairs)
    //                      .second;
    // _contact_groups =
    //     make_contact_group_storage(
    //         allocator,
    //         create_info.max_particle_particle_neighbor_pairs +
    //             create_info.max_particle_rigid_body_neighbor_pairs +
    //             create_info.max_particle_static_body_neighbor_pairs +
    //             create_info.max_rigid_body_rigid_body_neighbor_pairs +
    //             create_info.max_rigid_body_static_body_neighbor_pairs,
    //         create_info.max_contact_groups)
    //         .second;
    // _contact_group_fringe =
    //     make_dynamic_object_list(allocator,
    //                              create_info.max_contact_group_fringe_size)
    //         .second;
  }

  ~Impl() {
    // _contact_group_fringe = {};
    // _contact_groups = {};
    // _contact_cache = {};
    _rigid_body_static_body_contact_ptrs = {};
    _rigid_body_rigid_body_contact_ptrs = {};
    _particle_static_body_contact_ptrs = {};
    _particle_rigid_body_contact_ptrs = {};
    _particle_particle_contact_ptrs = {};
    // _rigid_body_static_body_contact_pairs = {};
    // _rigid_body_rigid_body_contact_pairs = {};
    _rigid_body_static_body_contacts = {};
    _rigid_body_rigid_body_contacts = {};
    _particle_static_body_contacts = {};
    _particle_rigid_body_contacts = {};
    _particle_particle_contacts = {};
    _neighbor_groups = {};
    _static_body_neighbors = {};
    _rigid_body_neighbors = {};
    _particle_neighbors = {};
    _rigid_body_static_body_neighbor_pairs = {};
    _rigid_body_rigid_body_neighbor_pairs = {};
    _particle_static_body_neighbor_pairs = {};
    _particle_rigid_body_neighbor_pairs = {};
    _particle_particle_neighbor_pairs = {};
    util::System_allocator::instance()->free(_block);
  }

  Particle_handle create_particle(Particle_create_info const &create_info) {
    auto const bounds =
        Aabb{create_info.position - Vec3f::all(create_info.radius),
             create_info.position + Vec3f::all(create_info.radius)};
    auto const particle = _particles.create({
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Particle_handle{}),
        .motion_callback = create_info.motion_callback,
        .radius = create_info.radius,
        .inverse_mass = 1.0f / create_info.mass,
        .material = create_info.material,
        .previous_position = create_info.position,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .waking_motion = waking_motion_initializer,
        .marked = false,
        .visited = false,
        .awake = true,
    });
    _particles.data(particle)->aabb_tree_node->payload = particle;
    return particle;
  }

  void destroy_particle(Particle_handle particle) {
    _aabb_tree.destroy_leaf(_particles.data(particle)->aabb_tree_node);
    _particles.destroy(particle);
  }

  bool is_awake(Particle_handle particle) const noexcept {
    return _particles.data(particle)->awake;
  }

  float get_waking_motion(Particle_handle particle) const noexcept {
    return _particles.data(particle)->waking_motion;
  }

  math::Vec3f get_position(Particle_handle particle) const noexcept {
    return _particles.data(particle)->position;
  }

  Rigid_body_handle
  create_rigid_body(Rigid_body_create_info const &create_info) {
    auto const transform =
        Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const bounds = physics::bounds(create_info.shape, transform);
    auto const rigid_body = _rigid_bodies.create({
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Rigid_body_handle{}),
        .motion_callback = create_info.motion_callback,
        .shape = create_info.shape,
        .inverse_mass = 1.0f / create_info.mass,
        .inverse_inertia_tensor = inverse(create_info.inertia_tensor),
        .material = create_info.material,
        .previous_position = create_info.position,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .previous_orientation = create_info.orientation,
        .orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
        .waking_motion = waking_motion_initializer,
        .marked = false,
        .visited = false,
        .awake = true,
    });
    _rigid_bodies.data(rigid_body)->aabb_tree_node->payload = rigid_body;
    return rigid_body;
  }

  void destroy_rigid_body(Rigid_body_handle rigid_body) {
    _aabb_tree.destroy_leaf(_rigid_bodies.data(rigid_body)->aabb_tree_node);
    _rigid_bodies.destroy(rigid_body);
  }

  bool is_awake(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body)->awake;
  }

  float get_waking_motion(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body)->waking_motion;
  }

  math::Vec3f get_position(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body)->position;
  }

  math::Quatf get_orientation(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body)->orientation;
  }

  Static_body_handle
  create_static_rigid_body(Static_body_create_info const &create_info) {
    auto const transform =
        Mat3x4f::rigid(create_info.position, create_info.orientation);
    auto const transform_inverse = rigid_inverse(transform);
    auto const bounds = physics::bounds(create_info.shape, transform);
    auto const static_body = _static_bodies.create({
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Static_body_handle{}),
        .shape = create_info.shape,
        .material = create_info.material,
        .transform = transform,
        .inverse_transform = transform_inverse,
    });
    _static_bodies.data(static_body)->aabb_tree_node->payload = static_body;
    return static_body;
  }

  void destroy_static_rigid_body(Static_body_handle handle) {
    _aabb_tree.destroy_leaf(_static_bodies.data(handle)->aabb_tree_node);
    _static_bodies.destroy(handle);
  }

  void simulate(World const &world, World_simulate_info const &simulate_info) {
    build_aabb_tree(simulate_info.delta_time);
    clear_neighbor_state();
    find_neighbor_pairs();
    allocate_neighbors();
    assign_neighbors();
    find_neighbor_groups();
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const time_compensated_velocity_damping_factor =
        std::pow(velocity_damping_factor, h);
    auto const time_compensating_waking_motion_smoothing_factor =
        1.0f - std::pow(1.0f - waking_motion_smoothing_factor, h);
    auto const gravitational_delta_velocity = _gravitational_acceleration * h;
    auto const restitution_max_separating_velocity =
        -2.0f * length(gravitational_delta_velocity);
    // _contact_cache.clear_unmarked();
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      _awake_neighbor_groups.clear();
      clear_contact_state();
      for (auto j = std::size_t{}; j != _neighbor_groups.group_count(); ++j) {
        if (update_neighbor_group_awake_states(j)) {
          _awake_neighbor_groups.emplace_back(j);
          integrate_neighbor_group(
              j,
              h,
              time_compensated_velocity_damping_factor,
              time_compensating_waking_motion_smoothing_factor);
          solve_neighbor_group_positions(j);
          // find_neighbor_group_contacts(j);
          // find_neighbor_group_contact_groups(j);
        }
      }
      assign_contact_ptrs();
      for (auto const j : _awake_neighbor_groups) {
        solve_neighbor_group_velocities(j,
                                        gravitational_delta_velocity,
                                        restitution_max_separating_velocity);
      }
      // auto latch = std::latch{
      //     static_cast<std::ptrdiff_t>(_contact_groups.group_count())};
      // auto const intrinsic_state = Contact_group_solve_task::Intrinsic_state{
      //     .latch = &latch,
      //     .particles = &_particles,
      //     .rigid_bodies = &_rigid_bodies,
      //     .static_bodies = &_static_bodies,
      //     .contact_groups = &_contact_groups,
      //     .min_desired_position_iterations_per_contact =
      //         simulate_info.min_desired_position_iterations_per_contact,
      //     .max_desired_position_iterations_per_contact =
      //         simulate_info.max_desired_position_iterations_per_contact,
      //     .max_position_iterations =
      //         simulate_info.max_position_iterations_per_contact_group,
      //     .min_desired_velocity_iterations_per_contact =
      //         simulate_info.min_desired_velocity_iterations_per_contact,
      //     .max_desired_velocity_iterations_per_contact =
      //         simulate_info.max_desired_velocity_iterations_per_contact,
      //     .max_velocity_iterations =
      //         simulate_info.max_velocity_iterations_per_contact_group,
      //     .early_out_contact_separation =
      //         simulate_info.early_out_contact_separation,
      //     .early_out_contact_separating_velocity =
      //         simulate_info.early_out_contact_separating_velocity,
      //     .gravitational_delta_velocity = gravitational_delta_velocity,
      //     .restitution_max_separating_velocity =
      //         restitution_max_separating_velocity,
      // };
      // for (auto j = std::size_t{}; j != _contact_groups.group_count(); ++j) {
      //   _contact_groups.group_solve_task(j)->set_instrinsic_state(
      //       &intrinsic_state);
      //   simulate_info.thread_pool->push(_contact_groups.group_solve_task(j));
      // }
      // latch.wait();
    }
    call_particle_motion_callbacks(world);
    call_dynamic_rigid_body_motion_callbacks(world);
  }

private:
  bool is_marked(Particle_handle particle) const {
    return _particles.data(particle)->marked;
  }

  bool is_marked(Rigid_body_handle rigid_body) const {
    return _rigid_bodies.data(rigid_body)->marked;
  }

  void set_marked(Particle_handle particle, bool marked = true) {
    _particles.data(particle)->marked = marked;
  }

  void set_marked(Rigid_body_handle rigid_body, bool marked = true) {
    _rigid_bodies.data(rigid_body)->marked = marked;
  }

  void set_unmarked(Particle_handle particle) { set_marked(particle, false); }

  void set_unmarked(Rigid_body_handle rigid_body) {
    set_marked(rigid_body, false);
  }

  bool is_visited(Particle_handle particle) const {
    return _particles.data(particle)->visited;
  }

  bool is_visited(Rigid_body_handle rigid_body) const {
    return _rigid_bodies.data(rigid_body)->visited;
  }

  void set_visited(Particle_handle particle, bool visited = true) {
    _particles.data(particle)->visited = visited;
  }

  void set_visited(Rigid_body_handle rigid_body, bool visited = true) {
    _rigid_bodies.data(rigid_body)->visited = visited;
  }

  void set_unvisited(Particle_handle particle) { set_visited(particle, false); }

  void set_unvisited(Rigid_body_handle rigid_body) {
    set_visited(rigid_body, false);
  }

  std::span<Particle_handle> get_particle_neighbors(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->particle_neighbors, data->particle_neighbor_count};
  }

  std::span<Particle_handle>
  get_particle_neighbors(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->particle_neighbors, data->particle_neighbor_count};
  }

  std::span<Rigid_body_handle>
  get_rigid_body_neighbors(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->rigid_body_neighbors, data->rigid_body_neighbor_count};
  }

  std::span<Rigid_body_handle>
  get_rigid_body_neighbors(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->rigid_body_neighbors, data->rigid_body_neighbor_count};
  }

  std::span<Static_body_handle>
  get_static_body_neighbors(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->static_body_neighbors, data->static_body_neighbor_count};
  }

  std::span<Static_body_handle>
  get_static_body_neighbors(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->static_body_neighbors, data->static_body_neighbor_count};
  }

  std::span<Particle_particle_contact *>
  get_particle_contacts(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->particle_contacts, data->particle_contact_count};
  }

  std::span<Particle_rigid_body_contact *>
  get_particle_contacts(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->particle_contacts, data->particle_contact_count};
  }

  std::span<Particle_rigid_body_contact *>
  get_rigid_body_contacts(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->rigid_body_contacts, data->rigid_body_contact_count};
  }

  std::span<Rigid_body_rigid_body_contact *>
  get_rigid_body_contacts(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->rigid_body_contacts, data->rigid_body_contact_count};
  }

  std::span<Particle_static_body_contact *>
  get_static_body_contacts(Particle_handle particle) {
    auto const data = _particles.data(particle);
    return {data->static_body_contacts, data->static_body_contact_count};
  }

  std::span<Rigid_body_static_body_contact *>
  get_static_body_contacts(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    return {data->static_body_contacts, data->static_body_contact_count};
  }

  void build_aabb_tree(float delta_time) {
    auto const constant_safety_term = 0.0f;
    auto const velocity_safety_factor = 2.0f;
    auto const gravity_safety_factor = 2.0f;
    auto const gravity_safety_term = gravity_safety_factor *
                                     length(_gravitational_acceleration) *
                                     delta_time * delta_time;
    _particles.for_each([&](Particle_handle, Particle_data *data) {
      auto const half_extents = Vec3f::all(
          data->radius + constant_safety_term +
          velocity_safety_factor * length(data->velocity) * delta_time +
          gravity_safety_term);
      data->aabb_tree_node->bounds = {data->position - half_extents,
                                      data->position + half_extents};
    });
    _rigid_bodies.for_each([&](Rigid_body_handle, Rigid_body_data *data) {
      data->aabb_tree_node->bounds = expand(
          bounds(data->shape,
                 Mat3x4f::rigid(data->position, data->orientation)),
          constant_safety_term +
              velocity_safety_factor * length(data->velocity) * delta_time +
              gravity_safety_term);
    });
    _aabb_tree.build();
  }

  void clear_neighbor_state() {
    reset_neighbor_counts();
    _particle_particle_neighbor_pairs.clear();
    _particle_rigid_body_neighbor_pairs.clear();
    _particle_static_body_neighbor_pairs.clear();
    _rigid_body_rigid_body_neighbor_pairs.clear();
    _rigid_body_static_body_neighbor_pairs.clear();
    _particle_neighbors.clear();
    _rigid_body_neighbors.clear();
    _static_body_neighbors.clear();
    _neighbor_groups.clear();
  }

  void reset_neighbor_counts() {
    _particles.for_each([](Particle_handle, Particle_data *data) {
      data->particle_neighbor_count = 0;
      data->rigid_body_neighbor_count = 0;
      data->static_body_neighbor_count = 0;
    });
    _rigid_bodies.for_each([](Rigid_body_handle, Rigid_body_data *data) {
      data->particle_neighbor_count = 0;
      data->rigid_body_neighbor_count = 0;
      data->static_body_neighbor_count = 0;
    });
  }

  void find_neighbor_pairs() {
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
                      _particle_particle_neighbor_pairs.push_back(
                          {first_handle, second_handle});
                      ++_particles.data(first_handle)->particle_neighbor_count;
                      ++_particles.data(second_handle)->particle_neighbor_count;
                    } else if constexpr (std::is_same_v<U, Rigid_body_handle>) {
                      _particle_rigid_body_neighbor_pairs.push_back(
                          {first_handle, second_handle});
                      ++_particles.data(first_handle)
                            ->rigid_body_neighbor_count;
                      ++_rigid_bodies.data(second_handle)
                            ->particle_neighbor_count;
                    } else {
                      static_assert(std::is_same_v<U, Static_body_handle>);
                      _particle_static_body_neighbor_pairs.push_back(
                          {first_handle, second_handle});
                      ++_particles.data(first_handle)
                            ->static_body_neighbor_count;
                    }
                  },
                  second_payload);
            } else if constexpr (std::is_same_v<T, Rigid_body_handle>) {
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_handle>) {
                      _particle_rigid_body_neighbor_pairs.push_back(
                          {second_handle, first_handle});
                      ++_particles.data(second_handle)
                            ->rigid_body_neighbor_count;
                      ++_rigid_bodies.data(first_handle)
                            ->particle_neighbor_count;
                    } else if constexpr (std::is_same_v<U, Rigid_body_handle>) {
                      auto handles = std::pair{first_handle, second_handle};
                      if (handles.first.value > handles.second.value) {
                        std::swap(handles.first, handles.second);
                      }
                      _rigid_body_rigid_body_neighbor_pairs.push_back(handles);
                      ++_rigid_bodies.data(first_handle)
                            ->rigid_body_neighbor_count;
                      ++_rigid_bodies.data(second_handle)
                            ->rigid_body_neighbor_count;
                      // _contact_cache.set_marked(handles);
                    } else {
                      static_assert(std::is_same_v<U, Static_body_handle>);
                      auto handles = std::pair{first_handle, second_handle};
                      _rigid_body_static_body_neighbor_pairs.push_back(handles);
                      ++_rigid_bodies.data(first_handle)
                            ->static_body_neighbor_count;
                      // _contact_cache.set_marked(handles);
                    }
                  },
                  second_payload);
            } else {
              static_assert(std::is_same_v<T, Static_body_handle>);
              std::visit(
                  [this, first_handle](auto &&second_handle) {
                    using U = std::decay_t<decltype(second_handle)>;
                    if constexpr (std::is_same_v<U, Particle_handle>) {
                      _particle_static_body_neighbor_pairs.push_back(
                          {second_handle, first_handle});
                      ++_particles.data(second_handle)
                            ->static_body_neighbor_count;
                    } else if constexpr (std::is_same_v<U, Rigid_body_handle>) {
                      auto handles = std::pair{second_handle, first_handle};
                      _rigid_body_static_body_neighbor_pairs.push_back(handles);
                      ++_rigid_bodies.data(second_handle)
                            ->static_body_neighbor_count;
                      // _contact_cache.set_marked(handles);
                    }
                  },
                  second_payload);
            }
          },
          first_payload);
    });
  }

  void allocate_neighbors() {
    auto const f = [this](auto const, auto const data) {
      data->particle_neighbors = _particle_neighbors.end();
      _particle_neighbors.resize(_particle_neighbors.size() +
                                 data->particle_neighbor_count);
      data->rigid_body_neighbors = _rigid_body_neighbors.end();
      _rigid_body_neighbors.resize(_rigid_body_neighbors.size() +
                                   data->rigid_body_neighbor_count);
      data->static_body_neighbors = _static_body_neighbors.end();
      _static_body_neighbors.resize(_static_body_neighbors.size() +
                                    data->static_body_neighbor_count);
    };
    _particles.for_each(f);
    _rigid_bodies.for_each(f);
  }

  void assign_neighbors() {
    reset_neighbor_counts();
    for (auto const &[handle_1, handle_2] : _particle_particle_neighbor_pairs) {
      auto const data_1 = _particles.data(handle_1);
      auto const data_2 = _particles.data(handle_2);
      data_1->particle_neighbors[data_1->particle_neighbor_count++] = handle_2;
      data_2->particle_neighbors[data_2->particle_neighbor_count++] = handle_1;
    }
    for (auto const &[particle_handle, rigid_body_handle] :
         _particle_rigid_body_neighbor_pairs) {
      auto const particle_data = _particles.data(particle_handle);
      auto const rigid_body_data = _rigid_bodies.data(rigid_body_handle);
      particle_data
          ->rigid_body_neighbors[particle_data->rigid_body_neighbor_count++] =
          rigid_body_handle;
      rigid_body_data
          ->particle_neighbors[rigid_body_data->particle_neighbor_count++] =
          particle_handle;
    }
    for (auto const &[particle_handle, static_body_handle] :
         _particle_static_body_neighbor_pairs) {
      auto const particle_data = _particles.data(particle_handle);
      particle_data
          ->static_body_neighbors[particle_data->static_body_neighbor_count++] =
          static_body_handle;
    }
    for (auto const &[handle_1, handle_2] :
         _rigid_body_rigid_body_neighbor_pairs) {
      auto const data_1 = _rigid_bodies.data(handle_1);
      auto const data_2 = _rigid_bodies.data(handle_2);
      data_1->rigid_body_neighbors[data_1->rigid_body_neighbor_count++] =
          handle_2;
      data_2->rigid_body_neighbors[data_2->rigid_body_neighbor_count++] =
          handle_1;
    }
    for (auto const &[rigid_body_handle, static_body_handle] :
         _rigid_body_static_body_neighbor_pairs) {
      auto const rigid_body_data = _rigid_bodies.data(rigid_body_handle);
      rigid_body_data->static_body_neighbors
          [rigid_body_data->static_body_neighbor_count++] = static_body_handle;
    }
  }

  void find_neighbor_groups() {
    _particles.for_each(
        [](Particle_handle, Particle_data *data) { data->marked = false; });
    _rigid_bodies.for_each(
        [](Rigid_body_handle, Rigid_body_data *data) { data->marked = false; });
    auto fringe_index = std::size_t{};
    auto visitor = [this](auto &&handle) {
      using T = std::decay_t<decltype(handle)>;
      if constexpr (std::is_same_v<T, Particle_handle>) {
        auto const data = _particles.data(handle);
        for (auto const neighbor_handle : std::span{
                 data->particle_neighbors, data->particle_neighbor_count}) {
          auto const neighbor_data = _particles.data(neighbor_handle);
          if (!neighbor_data->marked) {
            neighbor_data->marked = true;
            _neighbor_groups.add_to_group(neighbor_handle);
          }
        }
        for (auto const neighbor_handle : std::span{
                 data->rigid_body_neighbors, data->rigid_body_neighbor_count}) {
          auto const neighbor_data = _rigid_bodies.data(neighbor_handle);
          if (!neighbor_data->marked) {
            neighbor_data->marked = true;
            _neighbor_groups.add_to_group(neighbor_handle);
          }
        }
      } else {
        static_assert(std::is_same_v<T, Rigid_body_handle>);
        auto const data = _rigid_bodies.data(handle);
        for (auto const neighbor_handle : std::span{
                 data->particle_neighbors, data->particle_neighbor_count}) {
          auto const neighbor_data = _particles.data(neighbor_handle);
          if (!neighbor_data->marked) {
            neighbor_data->marked = true;
            _neighbor_groups.add_to_group(neighbor_handle);
          }
        }
        for (auto const neighbor_handle : std::span{
                 data->rigid_body_neighbors, data->rigid_body_neighbor_count}) {
          auto const neighbor_data = _rigid_bodies.data(neighbor_handle);
          if (!neighbor_data->marked) {
            neighbor_data->marked = true;
            _neighbor_groups.add_to_group(neighbor_handle);
          }
        }
      }
    };
    _particles.for_each([&, this](Particle_handle handle, Particle_data *data) {
      if (!data->marked) {
        data->marked = true;
        _neighbor_groups.begin_group();
        _neighbor_groups.add_to_group(handle);
        do {
          std::visit(visitor, _neighbor_groups.object(fringe_index));
        } while (++fringe_index != _neighbor_groups.object_count());
      }
    });
    _rigid_bodies.for_each(
        [&, this](Rigid_body_handle handle, Rigid_body_data *data) {
          if (!data->marked) {
            data->marked = true;
            _neighbor_groups.begin_group();
            _neighbor_groups.add_to_group(handle);
            do {
              std::visit(visitor, _neighbor_groups.object(fringe_index));
            } while (++fringe_index != _neighbor_groups.object_count());
          }
        });
  }

  void clear_contact_state() {
    _particle_particle_contacts.clear();
    _particle_rigid_body_contacts.clear();
    _particle_static_body_contacts.clear();
    _rigid_body_rigid_body_contacts.clear();
    _rigid_body_static_body_contacts.clear();
    _particle_particle_contact_ptrs.clear();
    _particle_rigid_body_contact_ptrs.clear();
    _particle_static_body_contact_ptrs.clear();
    _rigid_body_rigid_body_contact_ptrs.clear();
    _rigid_body_static_body_contact_ptrs.clear();
    // _contact_groups.clear();
  }

  bool update_neighbor_group_awake_states(std::size_t group_index) {
    auto const group_begin = _neighbor_groups.group_begin(group_index);
    auto const group_end = _neighbor_groups.group_end(group_index);
    auto contains_awake = false;
    auto contains_sleeping = false;
    auto sleepable = true;
    for (auto i = group_begin;
         (sleepable || !contains_awake || !contains_sleeping) && i != group_end;
         ++i) {
      auto const object = _neighbor_groups.object(i);
      std::visit(
          [&](auto &&handle) {
            using T = std::decay_t<decltype(handle)>;
            if constexpr (std::is_same_v<T, Particle_handle>) {
              auto const data = _particles.data(handle);
              if (data->awake) {
                contains_awake = true;
                if (data->waking_motion > waking_motion_epsilon) {
                  sleepable = false;
                }
              } else {
                contains_sleeping = true;
              }
            } else {
              static_assert(std::is_same_v<T, Rigid_body_handle>);
              auto const data = _rigid_bodies.data(handle);
              if (data->awake) {
                contains_awake = true;
                if (data->waking_motion > waking_motion_epsilon) {
                  sleepable = false;
                }
              } else {
                contains_sleeping = true;
              }
            }
          },
          object);
    }
    if (contains_awake) {
      if (sleepable) {
        for (auto i = group_begin; i != group_end; ++i) {
          auto const object = _neighbor_groups.object(i);
          std::visit(
              [&](auto &&handle) {
                using T = std::decay_t<decltype(handle)>;
                if constexpr (std::is_same_v<T, Particle_handle>) {
                  auto const data = _particles.data(handle);
                  if (data->awake) {
                    data->velocity = Vec3f::zero();
                    data->awake = false;
                  }
                } else {
                  static_assert(std::is_same_v<T, Rigid_body_handle>);
                  auto const data = _rigid_bodies.data(handle);
                  if (data->awake) {
                    data->velocity = Vec3f::zero();
                    data->angular_velocity = Vec3f::zero();
                    data->awake = false;
                  }
                }
              },
              object);
        }
        return false;
      } else {
        if (contains_sleeping) {
          for (auto i = group_begin; i != group_end; ++i) {
            auto const object = _neighbor_groups.object(i);
            std::visit(
                [&](auto &&handle) {
                  using T = std::decay_t<decltype(handle)>;
                  if constexpr (std::is_same_v<T, Particle_handle>) {
                    auto const data = _particles.data(handle);
                    if (!data->awake) {
                      data->waking_motion = waking_motion_initializer;
                      data->awake = true;
                    }
                  } else {
                    static_assert(std::is_same_v<T, Rigid_body_handle>);
                    auto const data = _rigid_bodies.data(handle);
                    if (!data->awake) {
                      data->waking_motion = waking_motion_initializer;
                      data->awake = true;
                    }
                  }
                },
                object);
          }
        }
        return true;
      }
    } else {
      return false;
    }
  }

  void integrate_neighbor_group(std::size_t group_index,
                                float delta_time,
                                float velocity_damping_factor,
                                float waking_motion_smoothing_factor) {
    auto const group_begin = _neighbor_groups.group_begin(group_index);
    auto const group_end = _neighbor_groups.group_end(group_index);
    for (auto i = group_begin; i != group_end; ++i) {
      std::visit(
          [&](auto &&object) {
            integrate(object,
                      delta_time,
                      velocity_damping_factor,
                      waking_motion_smoothing_factor);
          },
          _neighbor_groups.object(i));
    }
  }

  void integrate(Particle_handle particle,
                 float delta_time,
                 float velocity_damping_factor,
                 float waking_motion_smoothing_factor) {
    auto const data = _particles.data(particle);
    data->previous_position = data->position;
    data->velocity += delta_time * _gravitational_acceleration;
    data->velocity *= velocity_damping_factor;
    data->position += delta_time * data->velocity;
    data->waking_motion = std::min(
        (1.0f - waking_motion_smoothing_factor) * data->waking_motion +
            waking_motion_smoothing_factor * length_squared(data->velocity),
        waking_motion_limit);
  }

  void integrate(Rigid_body_handle rigid_body,
                 float delta_time,
                 float velocity_damping_factor,
                 float waking_motion_smoothing_factor) {
    auto const data = _rigid_bodies.data(rigid_body);
    data->previous_position = data->position;
    data->previous_orientation = data->orientation;
    data->velocity += delta_time * _gravitational_acceleration;
    data->velocity *= velocity_damping_factor;
    data->position += delta_time * data->velocity;
    data->angular_velocity *= velocity_damping_factor;
    data->orientation +=
        Quatf{0.0f, 0.5f * delta_time * data->angular_velocity} *
        data->orientation;
    data->orientation = normalize(data->orientation);
    data->waking_motion =
        std::min((1.0f - waking_motion_smoothing_factor) * data->waking_motion +
                     waking_motion_smoothing_factor *
                         (length_squared(data->velocity) +
                          length_squared(data->angular_velocity)),
                 waking_motion_limit);
  }

  // void find_neighbor_group_contacts(std::size_t group_index) {
  //   auto const group_begin = _neighbor_groups.group_begin(group_index);
  //   auto const group_end = _neighbor_groups.group_end(group_index);
  //   for (auto i = group_begin; i != group_end; ++i) {
  //     std::visit(
  //         [&](auto &&object) {
  //           set_unmarked(object);
  //           reset_contact_counts(object);
  //         },
  //         _neighbor_groups.object(i));
  //   }
  //   auto const particle_particle_contacts_begin =
  //       _particle_particle_contacts.data() +
  //       _particle_particle_contacts.size();
  //   auto const particle_rigid_body_contacts_begin =
  //       _particle_rigid_body_contacts.data() +
  //       _particle_rigid_body_contacts.size();
  //   auto const particle_static_body_contacts_begin =
  //       _particle_static_body_contacts.data() +
  //       _particle_static_body_contacts.size();
  //   auto const rigid_body_rigid_body_contact_pairs_begin =
  //       _rigid_body_rigid_body_contact_pairs.data() +
  //       _rigid_body_rigid_body_contact_pairs.size();
  //   auto const rigid_body_static_body_contact_pairs_begin =
  //       _rigid_body_static_body_contact_pairs.data() +
  //       _rigid_body_static_body_contact_pairs.size();
  //   for (auto i = group_begin; i != group_end; ++i) {
  //     std::visit(
  //         [&](auto &&object) {
  //           set_marked(object);
  //           for (auto const particle : get_particle_neighbors(object)) {
  //             if (!is_marked(particle)) {
  //               find_contact({object, particle});
  //             }
  //           }
  //           for (auto const rigid_body : get_rigid_body_neighbors(object)) {
  //             if (!is_marked(rigid_body)) {
  //               find_contact({object, rigid_body});
  //             }
  //           }
  //           for (auto const static_body : get_static_body_neighbors(object))
  //           {
  //             find_contact({object, static_body});
  //           }
  //           alloc_contact_ptrs(object);
  //           reset_contact_counts(object);
  //         },
  //         _neighbor_groups.object(i));
  //   }
  //   auto const particle_particle_contacts_end =
  //       _particle_particle_contacts.data() +
  //       _particle_particle_contacts.size();
  //   auto const particle_rigid_body_contacts_end =
  //       _particle_rigid_body_contacts.data() +
  //       _particle_rigid_body_contacts.size();
  //   auto const particle_static_body_contacts_end =
  //       _particle_static_body_contacts.data() +
  //       _particle_static_body_contacts.size();
  //   auto const rigid_body_rigid_body_contact_pairs_end =
  //       _rigid_body_rigid_body_contact_pairs.data() +
  //       _rigid_body_rigid_body_contact_pairs.size();
  //   auto const rigid_body_static_body_contact_pairs_end =
  //       _rigid_body_static_body_contact_pairs.data() +
  //       _rigid_body_static_body_contact_pairs.size();
  //   assign_contact_ptrs(
  //       {particle_particle_contacts_begin, particle_particle_contacts_end},
  //       {particle_rigid_body_contacts_begin,
  //       particle_rigid_body_contacts_end},
  //       {particle_static_body_contacts_begin,
  //        particle_static_body_contacts_end},
  //       {rigid_body_rigid_body_contact_pairs_begin,
  //        rigid_body_rigid_body_contact_pairs_end},
  //       {rigid_body_static_body_contact_pairs_begin,
  //        rigid_body_static_body_contact_pairs_end});
  // }

  void solve_neighbor_group_positions(std::size_t group_index) {
    auto const group_begin = _neighbor_groups.group_begin(group_index);
    auto const group_end = _neighbor_groups.group_end(group_index);
    for (auto i = group_begin; i != group_end; ++i) {
      std::visit(
          [&](auto &&object) {
            reset_contact_counts(object);
            set_unmarked(object);
          },
          _neighbor_groups.object(i));
    }
    for (auto i = group_begin; i != group_end; ++i) {
      std::visit(
          [&](auto &&object) {
            set_marked(object);
            for (auto const particle : get_particle_neighbors(object)) {
              if (!is_marked(particle)) {
                if (auto const contact = find_contact({object, particle})) {
                  resolve_contact_position(contact);
                }
              }
            }
            for (auto const rigid_body : get_rigid_body_neighbors(object)) {
              if (!is_marked(rigid_body)) {
                if (auto const contact = find_contact({object, rigid_body})) {
                  resolve_contact_position(contact);
                }
              }
            }
            for (auto const static_body : get_static_body_neighbors(object)) {
              if (auto const contact = find_contact({object, static_body})) {
                resolve_contact_position(contact);
              }
            }
            alloc_contact_ptrs(object);
            reset_contact_counts(object);
          },
          _neighbor_groups.object(i));
    }
  }

  void reset_contact_counts(Particle_handle particle) {
    auto const data = _particles.data(particle);
    data->particle_contact_count = 0;
    data->rigid_body_contact_count = 0;
    data->static_body_contact_count = 0;
  }

  void reset_contact_counts(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    data->particle_contact_count = 0;
    data->rigid_body_contact_count = 0;
    data->static_body_contact_count = 0;
  }

  Particle_particle_contact *
  find_contact(std::pair<Particle_handle, Particle_handle> objects) {
    auto const data = std::array<Particle_data *, 2>{
        _particles.data(objects.first), _particles.data(objects.second)};
    auto const displacement = data[0]->position - data[1]->position;
    auto const distance2 = length_squared(displacement);
    auto const contact_distance = data[0]->radius + data[1]->radius;
    auto const contact_distance2 = contact_distance * contact_distance;
    if (distance2 < contact_distance2) {
      auto const [normal, separation] = [&]() {
        if (distance2 == 0.0f) {
          // particles coincide, pick arbitrary contact normal
          Vec3f const contact_normal{1.0f, 0.0f, 0.0f};
          auto const separation = -contact_distance;
          return std::tuple{contact_normal, separation};
        } else {
          auto const distance = std::sqrt(distance2);
          auto const normal = displacement / distance;
          auto const separation = distance - contact_distance;
          return std::tuple{normal, separation};
        }
      }();
      // auto const separating_velocity =
      //     dot(data[0]->velocity - data[1]->velocity, normal);
      _particle_particle_contacts.push_back({normal,
                                             separation,
                                             //  separating_velocity,
                                             {objects.first, objects.second}});
      for (auto i = 0; i != 2; ++i) {
        ++data[i]->particle_contact_count;
      }
      return &_particle_particle_contacts.back();
    } else {
      return nullptr;
    }
  }

  Particle_rigid_body_contact *
  find_contact(std::pair<Particle_handle, Rigid_body_handle> objects) {
    auto const particle_data = _particles.data(objects.first);
    auto const body_data = _rigid_bodies.data(objects.second);
    auto const body_transform =
        Mat3x4f::rigid(body_data->position, body_data->orientation);
    auto const inverse_body_transform = rigid_inverse(body_transform);
    if (auto const contact_geometry =
            particle_shape_positionful_contact_geometry(
                particle_data->position,
                particle_data->radius,
                body_data->shape,
                body_transform,
                inverse_body_transform)) {
      auto const body_relative_position =
          contact_geometry->position - body_data->position;
      // auto const relative_velocity =
      //     particle_data->velocity -
      //     (body_data->velocity +
      //      cross(body_data->angular_velocity, body_relative_position));
      // auto const separating_velocity =
      //     dot(relative_velocity, contact_geometry->normal);
      _particle_rigid_body_contacts.push_back({contact_geometry->normal,
                                               contact_geometry->separation,
                                               //  separating_velocity,
                                               objects.first,
                                               objects.second,
                                               body_relative_position});
      ++particle_data->rigid_body_contact_count;
      ++body_data->particle_contact_count;
      // return true;
      return &_particle_rigid_body_contacts.back();
    } else {
      return nullptr;
    }
  }

  Particle_static_body_contact *
  find_contact(std::pair<Particle_handle, Static_body_handle> objects) {
    auto const particle_data = _particles.data(objects.first);
    auto const body_data = _static_bodies.data(objects.second);
    if (auto const contact_geometry =
            particle_shape_positionless_contact_geometry(
                particle_data->position,
                particle_data->radius,
                body_data->shape,
                body_data->transform,
                body_data->inverse_transform)) {
      // auto const separating_velocity =
      //     dot(particle_data->velocity, contact_geometry->normal);
      _particle_static_body_contacts.push_back({contact_geometry->normal,
                                                contact_geometry->separation,
                                                // separating_velocity,
                                                objects.first,
                                                objects.second});
      ++particle_data->static_body_contact_count;
      // return true;
      return &_particle_static_body_contacts.back();
    } else {
      return nullptr;
    }
  }

  Particle_rigid_body_contact *
  find_contact(std::pair<Rigid_body_handle, Particle_handle> objects) {
    return find_contact({objects.second, objects.first});
  }

  Rigid_body_rigid_body_contact *
  find_contact(std::pair<Rigid_body_handle, Rigid_body_handle> objects) {
    auto const data = std::array<Rigid_body_data *, 2>{
        _rigid_bodies.data(objects.first), _rigid_bodies.data(objects.second)};
    auto const body_transforms = std::array<Mat3x4f, 2>{
        Mat3x4f::rigid(data[0]->position, data[0]->orientation),
        Mat3x4f::rigid(data[1]->position, data[1]->orientation)};
    auto const body_inverse_transforms = std::array<Mat3x4f, 2>{
        rigid_inverse(body_transforms[0]), rigid_inverse(body_transforms[1])};
    if (auto const contact_geometry =
            shape_shape_contact_geometry(data[0]->shape,
                                         body_transforms[0],
                                         body_inverse_transforms[0],
                                         data[1]->shape,
                                         body_transforms[1],
                                         body_inverse_transforms[1])) {
      auto const body_relative_contact_positions =
          std::array<Vec3f, 2>{contact_geometry->position - data[0]->position,
                               contact_geometry->position - data[1]->position};
      // auto const relative_velocity =
      //     (data[0]->velocity + cross(data[0]->angular_velocity,
      //                                body_relative_contact_positions[0])) -
      //     (data[1]->velocity + cross(data[1]->angular_velocity,
      //                                body_relative_contact_positions[1]));
      // auto const separating_velocity =
      //     dot(relative_velocity, contact_geometry->normal);
      // auto const contact_count = _contact_cache.update(
      //     Rigid_body_rigid_body_contact{contact_geometry->normal,
      //                                   contact_geometry->separation,
      //                                   separating_velocity,
      //                                   {objects.first, objects.second},
      //                                   body_relative_contact_positions},
      //     _rigid_bodies);
      // for (auto i = 0; i != 2; ++i) {
      //   data[i]->rigid_body_contact_count += contact_count;
      // }
      // _rigid_body_rigid_body_contact_pairs.emplace_back(objects);
      _rigid_body_rigid_body_contacts.push_back(
          {contact_geometry->normal,
           contact_geometry->separation,
           {objects.first, objects.second},
           body_relative_contact_positions});
      for (auto i = 0; i != 2; ++i) {
        ++data[i]->rigid_body_contact_count;
      }
      return &_rigid_body_rigid_body_contacts.back();
    } else {
      // _contact_cache.clear(objects);
      // return false;
      return nullptr;
    }
  }

  Rigid_body_static_body_contact *
  find_contact(std::pair<Rigid_body_handle, Static_body_handle> objects) {
    auto const rigid_body_data = _rigid_bodies.data(objects.first);
    auto const rigid_body_transform =
        Mat3x4f::rigid(rigid_body_data->position, rigid_body_data->orientation);
    auto const rigid_body_inverse_transform =
        rigid_inverse(rigid_body_transform);
    auto const static_body_data = _static_bodies.data(objects.second);
    if (auto const contact_geometry =
            shape_shape_contact_geometry(rigid_body_data->shape,
                                         rigid_body_transform,
                                         rigid_body_inverse_transform,
                                         static_body_data->shape,
                                         static_body_data->transform,
                                         static_body_data->inverse_transform)) {
      auto const rigid_body_relative_contact_position =
          contact_geometry->position - rigid_body_data->position;
      // auto const relative_velocity =
      //     rigid_body_data->velocity +
      //     cross(rigid_body_data->angular_velocity,
      //           rigid_body_relative_contact_position);
      // auto const separating_velocity =
      //     dot(relative_velocity, contact_geometry->normal);
      // auto const contact_count = _contact_cache.update(
      //     Rigid_body_static_body_contact{contact_geometry->normal,
      //                                    contact_geometry->separation,
      //                                    separating_velocity,
      //                                    objects.first,
      //                                    objects.second,
      //                                    rigid_body_relative_contact_position},
      //     _rigid_bodies);
      // rigid_body_data->static_body_contact_count += contact_count;
      // _rigid_body_static_body_contact_pairs.emplace_back(objects);
      _rigid_body_static_body_contacts.push_back(
          {contact_geometry->normal,
           contact_geometry->separation,
           objects.first,
           objects.second,
           rigid_body_relative_contact_position});
      ++rigid_body_data->static_body_contact_count;
      return &_rigid_body_static_body_contacts.back();
    } else {
      // _contact_cache.clear(objects);
      return nullptr;
    }
  }

  void resolve_contact_position(Particle_particle_contact const *contact) {
    auto const particles = contact->particles;
    auto const particle_datas = std::array<Particle_data *, 2>{
        _particles.data(particles[0]),
        _particles.data(particles[1]),
    };
    auto const distance_per_impulse =
        particle_datas[0]->inverse_mass + particle_datas[1]->inverse_mass;
    auto const impulse_per_distance = 1.0f / distance_per_impulse;
    auto const impulse =
        -contact->separation * impulse_per_distance * contact->normal;
    auto const particle_position_deltas =
        std::array<Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
                             impulse * -particle_datas[1]->inverse_mass};
    particle_datas[0]->position += particle_position_deltas[0];
    particle_datas[1]->position += particle_position_deltas[1];
    // for (auto i = 0; i != 2; ++i) {
    //   update_separations(particles[i], particle_position_deltas[i]);
    // }
  }

  void resolve_contact_position(Particle_rigid_body_contact const *contact) {
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
    // update_separations(contact->particle, delta_position[0]);
    // update_separations(contact->body, delta_position[1], delta_orientation);
  }

  void resolve_contact_position(Particle_static_body_contact const *contact) {
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
    // update_separations(contact->particle, delta_position);
  }

  void resolve_contact_position(Rigid_body_rigid_body_contact const *contact) {
    auto const data = std::array<Rigid_body_data *, 2>{
        _rigid_bodies.data(contact->bodies[0]),
        _rigid_bodies.data(contact->bodies[1]),
    };
    auto const rotation = std::array<Mat3x3f, 2>{
        Mat3x3f::rotation(data[0]->orientation),
        Mat3x3f::rotation(data[1]->orientation),
    };
    auto const inverse_rotation = std::array<Mat3x3f, 2>{
        transpose(rotation[0]),
        transpose(rotation[1]),
    };
    auto const inverse_inertia_tensor = std::array<Mat3x3f, 2>{
        rotation[0] * data[0]->inverse_inertia_tensor * inverse_rotation[0],
        rotation[1] * data[1]->inverse_inertia_tensor * inverse_rotation[1],
    };
    auto const separation_solution = solve_positional_constraint({
        .direction = contact->normal,
        .distance = -contact->separation,
        .position = contact->relative_positions,
        .inverse_mass = {data[0]->inverse_mass, data[1]->inverse_mass},
        .inverse_inertia_tensor = inverse_inertia_tensor,
    });
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
    if (tangential_relative_contact_movement != Vec3f::zero()) {
      auto const correction_distance =
          length(tangential_relative_contact_movement);
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
      // update_separations(
      //     contact->bodies[i], delta_position[i], delta_orientation[i]);
    }
  }

  void resolve_contact_position(Rigid_body_static_body_contact const *contact) {
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
    // update_separations(contact->rigid_body, delta_position,
    // delta_orientation);
  }

  void apply_positional_correction(Particle_handle particle_handle,
                                   Vec3f const &delta_position) noexcept {
    auto const particle_data = _particles.data(particle_handle);
    particle_data->position += delta_position;
  }

  void apply_positional_correction(Rigid_body_handle body_handle,
                                   Vec3f const &delta_position,
                                   Vec3f const &delta_orientation) noexcept {
    auto const body_data = _rigid_bodies.data(body_handle);
    body_data->position += delta_position;
    body_data->orientation +=
        0.5f * Quatf{0.0f, delta_orientation} * body_data->orientation;
    body_data->orientation = normalize(body_data->orientation);
  }

  void assign_contact_ptrs() {
    for (auto &contact : _particle_particle_contacts) {
      for (auto i = 0; i != 2; ++i) {
        auto const data = _particles.data(contact.particles[i]);
        data->particle_contacts[data->particle_contact_count++] = &contact;
      }
    }
    for (auto &contact : _particle_rigid_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      auto const rigid_body_data = _rigid_bodies.data(contact.body);
      particle_data
          ->rigid_body_contacts[particle_data->rigid_body_contact_count++] =
          &contact;
      rigid_body_data
          ->particle_contacts[rigid_body_data->particle_contact_count++] =
          &contact;
    }
    for (auto &contact : _particle_static_body_contacts) {
      auto const particle_data = _particles.data(contact.particle);
      particle_data
          ->static_body_contacts[particle_data->static_body_contact_count++] =
          &contact;
    }
    for (auto &contact : _rigid_body_rigid_body_contacts) {
      auto const data = std::array<Rigid_body_data *, 2>{
          _rigid_bodies.data(contact.bodies[0]),
          _rigid_bodies.data(contact.bodies[1]),
      };
      for (auto i = 0; i != 2; ++i) {
        data[i]->rigid_body_contacts[data[i]->rigid_body_contact_count++] =
            &contact;
      }
    }
    for (auto &contact : _rigid_body_static_body_contacts) {
      auto const data = _rigid_bodies.data(contact.rigid_body);
      data->static_body_contacts[data->static_body_contact_count++] = &contact;
    }
  }

  void solve_neighbor_group_velocities(
      std::size_t group_index,
      math::Vec3f const &gravitational_delta_velocity,
      float restitution_max_separating_velocity) {
    auto const group_begin = _neighbor_groups.group_begin(group_index);
    auto const group_end = _neighbor_groups.group_end(group_index);
    for (auto i = group_begin; i != group_end; ++i) {
      std::visit([&](auto &&object) { set_unmarked(object); },
                 _neighbor_groups.object(i));
    }
    for (auto i = group_begin; i != group_end; ++i) {
      std::visit(
          [&](auto &&object) {
            using T = std::decay_t<decltype(object)>;
            set_marked(object);
            for (auto const contact : get_particle_contacts(object)) {
              if constexpr (std::is_same_v<T, Particle_handle>) {
                auto const particle =
                    contact->particles[contact->particles[0] == object];
                if (!is_marked(particle)) {
                  resolve_contact_velocity(contact,
                                           restitution_max_separating_velocity);
                }
              } else {
                auto const particle = contact->particle;
                if (!is_marked(particle)) {
                  resolve_contact_velocity(contact,
                                           restitution_max_separating_velocity);
                }
              }
            }
            for (auto const contact : get_rigid_body_contacts(object)) {
              if constexpr (std::is_same_v<T, Rigid_body_handle>) {
                auto const rigid_body =
                    contact->bodies[contact->bodies[0] == object];
                if (!is_marked(rigid_body)) {
                  resolve_contact_velocity(contact,
                                           restitution_max_separating_velocity);
                }
              } else {
                auto const rigid_body = contact->body;
                if (!is_marked(rigid_body)) {
                  resolve_contact_velocity(contact,
                                           restitution_max_separating_velocity);
                }
              }
            }
            for (auto const contact : get_static_body_contacts(object)) {
              resolve_contact_velocity(contact,
                                       gravitational_delta_velocity,
                                       restitution_max_separating_velocity);
              // if (auto const contact = find_contact({object, static_body})) {
              //   resolve_contact_velocity(contact,
              //                            gravitational_delta_velocity,
              //                            restitution_max_separating_velocity);
              // }
            }
            // alloc_contact_ptrs(object);
            // reset_contact_counts(object);
          },
          _neighbor_groups.object(i));
    }
  }

  void resolve_contact_velocity(Particle_particle_contact const *contact,
                                float max_separating_velocity_for_bounce) {
    auto const particles = contact->particles;
    auto const particle_datas = std::array<Particle_data *, 2>{
        _particles.data(particles[0]),
        _particles.data(particles[1]),
    };
    auto const normal = contact->normal;
    auto const separating_velocity =
        dot(particle_datas[0]->velocity - particle_datas[1]->velocity,
            contact->normal);
    // auto const separating_velocity = contact->separating_velocity;
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
      auto const sliding_speed_squared = length_squared(sliding_velocity);
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
        return Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
    auto const particle_velocity_deltas =
        std::array<Vec3f, 2>{impulse * particle_datas[0]->inverse_mass,
                             impulse * -particle_datas[1]->inverse_mass};
    particle_datas[0]->velocity += particle_velocity_deltas[0];
    particle_datas[1]->velocity += particle_velocity_deltas[1];
    // for (auto i = 0; i != 2; ++i) {
    //   update_separating_velocities(particles[i],
    //   particle_velocity_deltas[i]);
    // }
  }

  void resolve_contact_velocity(Particle_rigid_body_contact const *contact,
                                float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _rigid_bodies.data(body);
    auto const body_relative_contact_position = contact->relative_position;
    auto const normal = contact->normal;
    auto const separating_velocity =
        dot(particle_data->velocity -
                (body_data->velocity + cross(body_data->angular_velocity,
                                             contact->relative_position)),
            contact->normal);
    // auto const separating_velocity = contact->separating_velocity;
    auto const body_rotation = Mat3x3f::rotation(body_data->orientation);
    auto const body_inverse_rotation = transpose(body_rotation);
    auto const body_inverse_inertia_tensor = body_rotation *
                                             body_data->inverse_inertia_tensor *
                                             body_inverse_rotation;
    auto const body_angular_impulse_per_separating_impulse =
        cross(body_relative_contact_position, normal);
    auto const body_angular_velocity_per_separating_impulse =
        body_inverse_inertia_tensor *
        body_angular_impulse_per_separating_impulse;
    auto const separating_velocity_per_separating_impulse =
        particle_data->inverse_mass + body_data->inverse_mass +
        dot(cross(body_angular_velocity_per_separating_impulse,
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
        (body_data->velocity +
         cross(body_data->angular_velocity, body_relative_contact_position));
    auto const sliding_velocity =
        relative_velocity - separating_velocity * normal;
    auto const sliding_speed_squared = length_squared(sliding_velocity);
    auto const frictional_impulse = [&]() {
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const body_angular_impulse_per_frictional_impulse =
            cross(body_relative_contact_position, frictional_impulse_direction);
        auto const body_angular_velocity_per_frictional_impulse =
            body_inverse_inertia_tensor *
            body_angular_impulse_per_frictional_impulse;
        auto const sliding_velocity_per_frictional_impulse =
            particle_data->inverse_mass + body_data->inverse_mass +
            dot(cross(body_angular_velocity_per_frictional_impulse,
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
        return Vec3f::zero();
      }
    }();
    auto const impulse = separating_impulse + frictional_impulse;
    auto const particle_velocity_delta = impulse * particle_data->inverse_mass;
    auto const body_velocity_delta = -impulse * body_data->inverse_mass;
    auto const body_angular_velocity_delta =
        body_inverse_inertia_tensor *
        cross(body_relative_contact_position, -impulse);
    particle_data->velocity += particle_velocity_delta;
    body_data->velocity += body_velocity_delta;
    body_data->angular_velocity += body_angular_velocity_delta;
    // update_separating_velocities(particle, particle_velocity_delta);
    // update_separating_velocities(
    //     body, body_velocity_delta, body_angular_velocity_delta);
  }

  void resolve_contact_velocity(Particle_static_body_contact *contact,
                                Vec3f const &gravitational_velocity_delta,
                                float max_separating_velocity_for_bounce) {
    auto const particle = contact->particle;
    auto const particle_data = _particles.data(particle);
    auto const body = contact->body;
    auto const body_data = _static_bodies.data(body);
    auto const normal = contact->normal;
    auto const separating_velocity =
        dot(particle_data->velocity, contact->normal);
    // auto const separating_velocity = contact->separating_velocity;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (particle_data->material.restitution_coefficient +
                      body_data->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity -
        std::min(separating_velocity -
                     dot(gravitational_velocity_delta, normal),
                 0.0f) *
            restitution_coefficient;
    auto const relative_velocity = particle_data->velocity;
    auto const sliding_velocity =
        relative_velocity - separating_velocity * normal;
    auto const sliding_velocity_length_squared =
        length_squared(sliding_velocity);
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
            : Vec3f::zero();
    auto const particle_velocity_delta =
        delta_separating_velocity * normal + delta_sliding_velocity;
    particle_data->velocity += particle_velocity_delta;
    // update_separating_velocities(particle, particle_velocity_delta);
  }

  void resolve_contact_velocity(Rigid_body_rigid_body_contact *contact,
                                float max_separating_velocity_for_bounce) {
    auto const bodies = contact->bodies;
    auto const datas = std::array<Rigid_body_data *, 2>{
        _rigid_bodies.data(bodies[0]),
        _rigid_bodies.data(bodies[1]),
    };
    auto const normal = contact->normal;
    auto const relative_positions = contact->relative_positions;
    auto const relative_velocity =
        (datas[0]->velocity +
         cross(datas[0]->angular_velocity, relative_positions[0])) -
        (datas[1]->velocity +
         cross(datas[1]->angular_velocity, relative_positions[1]));
    auto const separating_velocity = dot(relative_velocity, normal);
    // auto const separating_velocity = contact->separating_velocity;
    auto const rotations = std::array<Mat3x3f, 2>{
        Mat3x3f::rotation(datas[0]->orientation),
        Mat3x3f::rotation(datas[1]->orientation),
    };
    auto const inverse_rotations = std::array<Mat3x3f, 2>{
        transpose(rotations[0]),
        transpose(rotations[1]),
    };
    auto const inverse_inertia_tensors = std::array<Mat3x3f, 2>{
        rotations[0] * datas[0]->inverse_inertia_tensor * inverse_rotations[0],
        rotations[1] * datas[1]->inverse_inertia_tensor * inverse_rotations[1],
    };
    auto const angular_impulses_per_separating_impulse = std::array<Vec3f, 2>{
        cross(relative_positions[0], normal),
        cross(relative_positions[1], normal),
    };
    auto const angular_velocities_per_separating_impulse = std::array<Vec3f, 2>{
        inverse_inertia_tensors[0] * angular_impulses_per_separating_impulse[0],
        inverse_inertia_tensors[1] * angular_impulses_per_separating_impulse[1],
    };
    auto const separating_velocity_per_separating_impulse =
        datas[0]->inverse_mass + datas[1]->inverse_mass +
        dot(cross(angular_velocities_per_separating_impulse[0],
                  relative_positions[0]) +
                cross(angular_velocities_per_separating_impulse[1],
                      relative_positions[1]),
            normal);
    auto const separating_impulse_per_separating_velocity =
        1.0f / separating_velocity_per_separating_impulse;
    auto const restitution_coefficient =
        separating_velocity < max_separating_velocity_for_bounce
            ? 0.5f * (datas[0]->material.restitution_coefficient +
                      datas[1]->material.restitution_coefficient)
            : 0.0f;
    auto const delta_separating_velocity =
        -separating_velocity * (1.0f + restitution_coefficient);
    auto const separating_impulse_length =
        delta_separating_velocity * separating_impulse_per_separating_velocity;
    auto const separating_impulse = separating_impulse_length * normal;
    auto const frictional_impulse = [&]() {
      auto const sliding_velocity =
          relative_velocity - separating_velocity * normal;
      auto const sliding_speed_squared = length_squared(sliding_velocity);
      if (sliding_speed_squared != 0.0f) {
        auto const sliding_speed = std::sqrt(sliding_speed_squared);
        auto const sliding_direction = sliding_velocity / sliding_speed;
        auto const frictional_impulse_direction = -sliding_direction;
        auto const angular_impulses_per_frictional_impulse =
            std::array<Vec3f, 2>{
                cross(relative_positions[0], frictional_impulse_direction),
                cross(relative_positions[1], frictional_impulse_direction),
            };
        auto const angular_velocities_per_frictional_impulse =
            std::array<Vec3f, 2>{
                inverse_inertia_tensors[0] *
                    angular_impulses_per_frictional_impulse[0],
                inverse_inertia_tensors[1] *
                    angular_impulses_per_frictional_impulse[1],
            };
        auto const sliding_velocity_per_frictional_impulse =
            datas[0]->inverse_mass + datas[1]->inverse_mass +
            dot(cross(angular_velocities_per_frictional_impulse[0],
                      relative_positions[0]),
                cross(angular_velocities_per_frictional_impulse[1],
                      relative_positions[1]));
        auto const frictional_impulse_per_sliding_velocity =
            1.0f / sliding_velocity_per_frictional_impulse;
        auto const static_friction_coefficient =
            0.5f * (datas[0]->material.static_friction_coefficient +
                    datas[1]->material.static_friction_coefficient);
        auto const dynamic_friction_coefficient =
            0.5f * (datas[0]->material.dynamic_friction_coefficient +
                    datas[1]->material.dynamic_friction_coefficient);
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
    auto const body_velocity_deltas = std::array<Vec3f, 2>{
        impulse * datas[0]->inverse_mass,
        -impulse * datas[1]->inverse_mass,
    };
    auto const body_angular_velocity_deltas = std::array<Vec3f, 2>{
        inverse_inertia_tensors[0] * cross(relative_positions[0], impulse),
        inverse_inertia_tensors[1] * cross(relative_positions[1], -impulse),
    };
    for (auto i = 0; i != 2; ++i) {
      datas[i]->velocity += body_velocity_deltas[i];
      datas[i]->angular_velocity += body_angular_velocity_deltas[i];
      // update_separating_velocities(
      //     bodies[i], body_velocity_deltas[i],
      //     body_angular_velocity_deltas[i]);
    }
  }

  void resolve_contact_velocity(Rigid_body_static_body_contact *contact,
                                Vec3f const &gravitational_velocity_delta,
                                float max_separating_velocity_for_bounce) {
    auto const dynamic_body = contact->rigid_body;
    auto const dynamic_body_data = _rigid_bodies.data(dynamic_body);
    auto const static_body = contact->static_body;
    auto const static_body_data = _static_bodies.data(static_body);
    auto const normal = contact->normal;
    auto const dynamic_body_relative_contact_position =
        contact->relative_position;
    auto const relative_velocity =
        dynamic_body_data->velocity +
        cross(dynamic_body_data->angular_velocity,
              dynamic_body_relative_contact_position);
    auto const separating_velocity = dot(relative_velocity, normal);
    auto const dynamic_body_rotation =
        Mat3x3f::rotation(dynamic_body_data->orientation);
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
        cross(dynamic_body_relative_contact_position, impulse);
    dynamic_body_data->velocity += dynamic_body_velocity_delta;
    dynamic_body_data->angular_velocity += dynamic_body_angular_velocity_delta;
    // update_separating_velocities(dynamic_body,
    //                              dynamic_body_velocity_delta,
    //                              dynamic_body_angular_velocity_delta);
  }

  void alloc_contact_ptrs(Particle_handle particle) {
    auto const data = _particles.data(particle);
    data->particle_contacts = _particle_particle_contact_ptrs.end();
    data->rigid_body_contacts = _particle_rigid_body_contact_ptrs.end();
    data->static_body_contacts = _particle_static_body_contact_ptrs.end();
    _particle_particle_contact_ptrs.resize(
        _particle_particle_contact_ptrs.size() + data->particle_contact_count);
    _particle_rigid_body_contact_ptrs.resize(
        _particle_rigid_body_contact_ptrs.size() +
        data->rigid_body_contact_count);
    _particle_static_body_contact_ptrs.resize(
        _particle_static_body_contact_ptrs.size() +
        data->static_body_contact_count);
  }

  void alloc_contact_ptrs(Rigid_body_handle rigid_body) {
    auto const data = _rigid_bodies.data(rigid_body);
    data->particle_contacts = _particle_rigid_body_contact_ptrs.end();
    data->rigid_body_contacts = _rigid_body_rigid_body_contact_ptrs.end();
    data->static_body_contacts = _rigid_body_static_body_contact_ptrs.end();
    _particle_rigid_body_contact_ptrs.resize(
        _particle_rigid_body_contact_ptrs.size() +
        data->particle_contact_count);
    _rigid_body_rigid_body_contact_ptrs.resize(
        _rigid_body_rigid_body_contact_ptrs.size() +
        data->rigid_body_contact_count);
    _rigid_body_static_body_contact_ptrs.resize(
        _rigid_body_static_body_contact_ptrs.size() +
        data->static_body_contact_count);
  }

  // void assign_contact_ptrs(
  //     std::span<Particle_particle_contact> particle_particle_contacts,
  //     std::span<Particle_rigid_body_contact> particle_rigid_body_contacts,
  //     std::span<Particle_static_body_contact> particle_static_body_contacts,
  //     std::span<std::pair<Rigid_body_handle, Rigid_body_handle> const>
  //         rigid_body_rigid_body_contact_pairs,
  //     std::span<std::pair<Rigid_body_handle, Static_body_handle> const>
  //         rigid_body_static_body_contact_pairs) {
  //   for (auto &contact : particle_particle_contacts) {
  //     for (auto i = 0; i != 2; ++i) {
  //       auto const data = _particles.data(contact.particles[i]);
  //       data->particle_contacts[data->particle_contact_count++] = &contact;
  //     }
  //   }
  //   for (auto &contact : particle_rigid_body_contacts) {
  //     auto const particle_data = _particles.data(contact.particle);
  //     auto const rigid_body_data = _rigid_bodies.data(contact.body);
  //     particle_data
  //         ->rigid_body_contacts[particle_data->rigid_body_contact_count++] =
  //         &contact;
  //     rigid_body_data
  //         ->particle_contacts[rigid_body_data->particle_contact_count++] =
  //         &contact;
  //   }
  //   for (auto &contact : particle_static_body_contacts) {
  //     auto const particle_data = _particles.data(contact.particle);
  //     particle_data
  //         ->static_body_contacts[particle_data->static_body_contact_count++]
  //         = &contact;
  //   }
  //   for (auto const &pair : rigid_body_rigid_body_contact_pairs) {
  //     for (auto &contact : _contact_cache.get_contacts(pair)) {
  //       auto const data = std::array<Rigid_body_data *, 2>{
  //           _rigid_bodies.data(contact.bodies[0]),
  //           _rigid_bodies.data(contact.bodies[1]),
  //       };
  //       for (auto i = 0; i != 2; ++i) {
  //         data[i]->rigid_body_contacts[data[i]->rigid_body_contact_count++] =
  //             &contact;
  //       }
  //     }
  //   }
  //   for (auto const &pair : rigid_body_static_body_contact_pairs) {
  //     for (auto &contact : _contact_cache.get_contacts(pair)) {
  //       auto const data = _rigid_bodies.data(contact.rigid_body);
  //       data->static_body_contacts[data->static_body_contact_count++] =
  //           &contact;
  //     }
  //   }
  // }

  // void find_neighbor_group_contact_groups(std::size_t group_index) {
  //   auto const group_begin = _neighbor_groups.group_begin(group_index);
  //   auto const group_end = _neighbor_groups.group_end(group_index);
  //   for (auto i = group_begin; i != group_end; ++i) {
  //     std::visit(
  //         [&](auto &&object) {
  //           set_unmarked(object);
  //           set_unvisited(object);
  //         },
  //         _neighbor_groups.object(i));
  //   }
  //   for (auto i = group_begin; i != group_end; ++i) {
  //     std::visit(
  //         [&, this](auto &&seed_object) {
  //           if (!is_marked(seed_object)) {
  //             find_contact_group(seed_object);
  //           }
  //         },
  //         _neighbor_groups.object(i));
  //   }
  // }

  // void find_contact_group(
  //     std::variant<Particle_handle, Rigid_body_handle> seed_object) {
  //   std::visit(
  //       [&](auto &&seed_object) {
  //         set_marked(seed_object);
  //         _contact_groups.begin_group();
  //         _contact_group_fringe.clear();
  //         _contact_group_fringe.push_back(seed_object);
  //       },
  //       seed_object);
  //   do {
  //     auto const fringe_object = _contact_group_fringe.back();
  //     _contact_group_fringe.pop_back();
  //     std::visit(
  //         [&](auto &&fringe_object) {
  //           using T = std::decay_t<decltype(fringe_object)>;
  //           set_visited(fringe_object);
  //           for (auto const contact : get_particle_contacts(fringe_object)) {
  //             if constexpr (std::is_same_v<T, Particle_handle>) {
  //               auto const other_object =
  //                   contact->particles[contact->particles[0] ==
  //                   fringe_object];
  //               if (!is_marked(other_object)) {
  //                 set_marked(other_object);
  //                 _contact_group_fringe.push_back(other_object);
  //               }
  //               if (!is_visited(other_object)) {
  //                 _contact_groups.add_to_group(contact);
  //               }
  //             } else {
  //               auto const other_object = contact->particle;
  //               if (!is_marked(other_object)) {
  //                 set_marked(other_object);
  //                 _contact_group_fringe.push_back(other_object);
  //               }
  //               if (!is_visited(other_object)) {
  //                 _contact_groups.add_to_group(contact);
  //               }
  //             }
  //           }
  //           for (auto const contact : get_rigid_body_contacts(fringe_object))
  //           {
  //             if constexpr (std::is_same_v<T, Rigid_body_handle>) {
  //               auto const other_object =
  //                   contact->bodies[contact->bodies[0] == fringe_object];
  //               if (!is_marked(other_object)) {
  //                 set_marked(other_object);
  //                 _contact_group_fringe.push_back(other_object);
  //               }
  //               if (!is_visited(other_object)) {
  //                 _contact_groups.add_to_group(contact);
  //               }
  //             } else {
  //               auto const other_object = contact->body;
  //               if (!is_marked(other_object)) {
  //                 set_marked(other_object);
  //                 _contact_group_fringe.push_back(other_object);
  //               }
  //               if (!is_visited(other_object)) {
  //                 _contact_groups.add_to_group(contact);
  //               }
  //             }
  //           }
  //           for (auto const contact :
  //           get_static_body_contacts(fringe_object)) {
  //             _contact_groups.add_to_group(contact);
  //           }
  //         },
  //         fringe_object);
  //   } while (!_contact_group_fringe.empty());
  // }

  void call_particle_motion_callbacks(World const &world) {
    _particles.for_each([&](Particle_handle particle, Particle_data *data) {
      if (data->motion_callback != nullptr) {
        data->motion_callback->on_particle_motion(world, particle);
      }
    });
  }

  void call_dynamic_rigid_body_motion_callbacks(World const &world) {
    _rigid_bodies.for_each(
        [&](Rigid_body_handle rigid_body, Rigid_body_data *data) {
          if (data->motion_callback != nullptr) {
            data->motion_callback->on_rigid_body_motion(world, rigid_body);
          }
        });
  }

  Block _block;
  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  Particle_storage _particles;
  Static_body_storage _static_bodies;
  Rigid_body_storage _rigid_bodies;
  List<std::pair<Particle_handle, Particle_handle>>
      _particle_particle_neighbor_pairs;
  List<std::pair<Particle_handle, Rigid_body_handle>>
      _particle_rigid_body_neighbor_pairs;
  List<std::pair<Particle_handle, Static_body_handle>>
      _particle_static_body_neighbor_pairs;
  List<std::pair<Rigid_body_handle, Rigid_body_handle>>
      _rigid_body_rigid_body_neighbor_pairs;
  List<std::pair<Rigid_body_handle, Static_body_handle>>
      _rigid_body_static_body_neighbor_pairs;
  List<Particle_handle> _particle_neighbors;
  List<Rigid_body_handle> _rigid_body_neighbors;
  List<Static_body_handle> _static_body_neighbors;
  Neighbor_group_storage _neighbor_groups;
  List<std::size_t> _awake_neighbor_groups;
  List<Particle_particle_contact> _particle_particle_contacts;
  List<Particle_rigid_body_contact> _particle_rigid_body_contacts;
  List<Particle_static_body_contact> _particle_static_body_contacts;
  List<Rigid_body_rigid_body_contact> _rigid_body_rigid_body_contacts;
  List<Rigid_body_static_body_contact> _rigid_body_static_body_contacts;
  // List<std::pair<Rigid_body_handle, Rigid_body_handle>>
  //     _rigid_body_rigid_body_contact_pairs;
  // List<std::pair<Rigid_body_handle, Static_body_handle>>
  //     _rigid_body_static_body_contact_pairs;
  List<Particle_particle_contact *> _particle_particle_contact_ptrs;
  List<Particle_rigid_body_contact *> _particle_rigid_body_contact_ptrs;
  List<Particle_static_body_contact *> _particle_static_body_contact_ptrs;
  List<Rigid_body_rigid_body_contact *> _rigid_body_rigid_body_contact_ptrs;
  List<Rigid_body_static_body_contact *> _rigid_body_static_body_contact_ptrs;
  // Contact_cache _contact_cache;
  // Contact_group_storage _contact_groups;
  // Dynamic_object_list _contact_group_fringe;
  Vec3f _gravitational_acceleration;
};

World::World(World_create_info const &create_info)
    : _impl{std::make_unique<Impl>(create_info)} {}

World::~World() {}

Particle_handle
World::create_particle(Particle_create_info const &create_info) {
  return _impl->create_particle(create_info);
}

void World::destroy_particle(Particle_handle particle) {
  _impl->destroy_particle(particle);
}

bool World::is_awake(Particle_handle particle) const noexcept {
  return _impl->is_awake(particle);
}

float World::get_waking_motion(Particle_handle particle) const noexcept {
  return _impl->get_waking_motion(particle);
}

math::Vec3f World::get_position(Particle_handle particle) const noexcept {
  return _impl->get_position(particle);
}

Rigid_body_handle
World::create_rigid_body(Rigid_body_create_info const &create_info) {
  return _impl->create_rigid_body(create_info);
}

void World::destroy_rigid_body(Rigid_body_handle handle) {
  _impl->destroy_rigid_body(handle);
}

bool World::is_awake(Rigid_body_handle rigid_body) const noexcept {
  return _impl->is_awake(rigid_body);
}

float World::get_waking_motion(Rigid_body_handle rigid_body) const noexcept {
  return _impl->get_waking_motion(rigid_body);
}

math::Vec3f World::get_position(Rigid_body_handle rigid_body) const noexcept {
  return _impl->get_position(rigid_body);
}

math::Quatf
World::get_orientation(Rigid_body_handle rigid_body) const noexcept {
  return _impl->get_orientation(rigid_body);
}

Static_body_handle
World::create_static_body(Static_body_create_info const &create_info) {
  return _impl->create_static_rigid_body(create_info);
}

void World::destroy_static_body(Static_body_handle static_rigid_body) {
  _impl->destroy_static_rigid_body(static_rigid_body);
}

void World::simulate(World_simulate_info const &simulate_info) {
  return _impl->simulate(*this, simulate_info);
}
} // namespace physics
} // namespace marlon