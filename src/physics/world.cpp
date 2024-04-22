#include "world.h"

#include <cstdint>

#include <latch>

#include "../math/scalar.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "../util/map.h"
#include "aabb_tree.h"

namespace marlon {
using math::Mat3x3f;
using math::Mat3x4f;
using math::Quatf;
using math::Vec3f;

using math::abs;
using math::max;
using math::min;
using math::pow;

using util::Bit_list;
using util::Block;
using util::Capacity_error;
using util::Lifetime_box;
using util::List;
using util::Map;
using util::Queue;
using util::Stack_allocator;

using util::make_block;
namespace physics {

namespace {
using Aabb_tree_payload_t =
    std::variant<Particle_handle, Static_body_handle, Rigid_body_handle>;

enum class Object_type : std::uint8_t { particle, rigid_body, static_body };

enum class Object_pair_type : std::uint8_t {
  particle_particle,
  particle_rigid_body,
  particle_static_body,
  rigid_body_rigid_body,
  rigid_body_static_body
};

auto constexpr color_unmarked{static_cast<std::uint16_t>(-1)};
auto constexpr color_marked{static_cast<std::uint16_t>(-2)};
auto constexpr reserved_colors{std::size_t{2}};
auto constexpr max_colors = (std::size_t{1} << 16) - reserved_colors;

struct Neighbor_pair {
  explicit Neighbor_pair(
      std::pair<Particle_handle, Particle_handle> objects) noexcept
      : objects{objects.first.value, objects.second.value},
        type{Object_pair_type::particle_particle} {}

  explicit Neighbor_pair(
      std::pair<Particle_handle, Rigid_body_handle> objects) noexcept
      : objects{objects.first.value, objects.second.value},
        type{Object_pair_type::particle_rigid_body} {}

  explicit Neighbor_pair(
      std::pair<Particle_handle, Static_body_handle> objects) noexcept
      : objects{objects.first.value, objects.second.value},
        type{Object_pair_type::particle_static_body} {}

  explicit Neighbor_pair(
      std::pair<Rigid_body_handle, Particle_handle> objects) noexcept
      : Neighbor_pair{{objects.second, objects.first}} {}

  explicit Neighbor_pair(
      std::pair<Rigid_body_handle, Rigid_body_handle> objects) noexcept
      : objects{objects.first.value, objects.second.value},
        type{Object_pair_type::rigid_body_rigid_body} {}

  explicit Neighbor_pair(
      std::pair<Rigid_body_handle, Static_body_handle> objects) noexcept
      : objects{objects.first.value, objects.second.value},
        type{Object_pair_type::rigid_body_static_body} {}

  explicit Neighbor_pair(
      std::pair<Static_body_handle, Particle_handle> objects) noexcept
      : Neighbor_pair{{objects.second, objects.first}} {}

  explicit Neighbor_pair(
      std::pair<Static_body_handle, Rigid_body_handle> objects) noexcept
      : Neighbor_pair{{objects.second, objects.first}} {}

  std::variant<Particle_handle, Rigid_body_handle> first() const noexcept {
    switch (type) {
    case Object_pair_type::particle_particle:
    case Object_pair_type::particle_rigid_body:
    case Object_pair_type::particle_static_body:
      return Particle_handle{objects[0]};
    case Object_pair_type::rigid_body_rigid_body:
    case Object_pair_type::rigid_body_static_body:
      return Rigid_body_handle{objects[0]};
    default:
      math::unreachable();
    }
  }

  std::variant<Particle_handle, Rigid_body_handle, Static_body_handle>
  second() const noexcept {
    switch (type) {
    case Object_pair_type::particle_particle:
      return Particle_handle{objects[1]};
    case Object_pair_type::particle_rigid_body:
      return Rigid_body_handle{objects[1]};
    case Object_pair_type::particle_static_body:
      return Static_body_handle{objects[1]};
    case Object_pair_type::rigid_body_rigid_body:
      return Rigid_body_handle{objects[1]};
    case Object_pair_type::rigid_body_static_body:
      return Static_body_handle{objects[1]};
    default:
      math::unreachable();
    }
  }

  std::variant<Particle_handle, Rigid_body_handle, Static_body_handle>
  other(Particle_handle object) const noexcept {
    switch (type) {
    case Object_pair_type::particle_particle:
      return Particle_handle{objects[objects[0] == object.value ? 1 : 0]};
    case Object_pair_type::particle_rigid_body:
      return Rigid_body_handle{objects[1]};
    case Object_pair_type::particle_static_body:
      return Static_body_handle{objects[1]};
    default:
      math::unreachable();
    }
  }

  std::variant<Particle_handle, Rigid_body_handle, Static_body_handle>
  other(Rigid_body_handle object) const noexcept {
    switch (type) {
    case Object_pair_type::particle_rigid_body:
      return Particle_handle{objects[0]};
    case Object_pair_type::rigid_body_rigid_body:
      return Rigid_body_handle{objects[objects[0] == object.value ? 1 : 0]};
    case Object_pair_type::rigid_body_static_body:
      return Static_body_handle{objects[1]};
    default:
      math::unreachable();
    }
  }

  std::array<std::uint32_t, 2> objects;
  Object_pair_type type;
  std::uint16_t color{color_unmarked};
};

struct Particle_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node{};
  Neighbor_pair **neighbor_pairs{};
  Particle_motion_callback *motion_callback{};
  float radius{};
  float inverse_mass{};
  Material material{};
  Vec3f previous_position{};
  Vec3f position{};
  Vec3f velocity{};
  float waking_motion{};
  std::uint16_t neighbor_count{};
  bool marked{};
  bool awake{};
};

struct Rigid_body_data {
  Aabb_tree<Aabb_tree_payload_t>::Node *aabb_tree_node{};
  Neighbor_pair **neighbor_pairs{};
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
  std::uint16_t neighbor_count{};
  bool marked{};
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
  using Allocator = Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<Block, Particle_storage> make(Allocator &allocator,
                                                 std::size_t max_particles) {
    auto const block = allocator.alloc(memory_requirement(max_particles));
    return {block, Particle_storage{block, max_particles}};
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_particles) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_particles),
        decltype(_available_handles)::memory_requirement(max_particles),
        decltype(_occupancy_bits)::memory_requirement(max_particles),
    });
  }

  constexpr Particle_storage() noexcept = default;

  explicit Particle_storage(Block block, std::size_t max_particles) noexcept
      : Particle_storage{block.begin, max_particles} {}

  explicit Particle_storage(void *block, std::size_t max_particles) noexcept {
    auto allocator =
        Allocator{make_block(block, memory_requirement(max_particles))};
    _data = decltype(_data)::make(allocator, max_particles).second;
    _data.resize(max_particles);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_particles).second;
    _available_handles.resize(max_particles);
    for (auto i = std::size_t{}; i != max_particles; ++i) {
      _available_handles[i] = {
          static_cast<std::uint32_t>(max_particles - i - 1)};
    }
    _occupancy_bits = Bit_list::make(allocator, max_particles).second;
    _occupancy_bits.resize(max_particles);
  }

  Particle_handle create(Particle_data const &data) {
    if (_available_handles.empty()) {
      throw Capacity_error{"Capacity_error in Particle_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.value].construct(data);
    _occupancy_bits.set(result.value);
    return result;
  }

  void destroy(Particle_handle particle) {
    _available_handles.emplace_back(particle);
    _occupancy_bits.reset(particle.value);
  }

  Particle_data const *data(Particle_handle particle) const noexcept {
    return _data[particle.value].get();
  }

  Particle_data *data(Particle_handle particle) noexcept {
    return _data[particle.value].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        auto const handle = Particle_handle{static_cast<std::uint32_t>(i)};
        f(handle, data(handle));
        ++k;
      }
    }
  }

private:
  List<Lifetime_box<Particle_data>> _data;
  List<Particle_handle> _available_handles;
  Bit_list _occupancy_bits;
};

class Rigid_body_storage {
  using Allocator = Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<Block, Rigid_body_storage>
  make(Allocator &allocator, std::size_t max_rigid_bodies) {
    auto const block = allocator.alloc(memory_requirement(max_rigid_bodies));
    return {block, Rigid_body_storage{block, max_rigid_bodies}};
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_rigid_bodies) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_rigid_bodies),
        decltype(_available_handles)::memory_requirement(max_rigid_bodies),
        decltype(_occupancy_bits)::memory_requirement(max_rigid_bodies),
    });
  }

  constexpr Rigid_body_storage() = default;

  explicit Rigid_body_storage(Block block,
                              std::size_t max_rigid_bodies) noexcept
      : Rigid_body_storage{block.begin, max_rigid_bodies} {}

  explicit Rigid_body_storage(void *block,
                              std::size_t max_rigid_bodies) noexcept {
    auto allocator =
        Allocator{make_block(block, memory_requirement(max_rigid_bodies))};
    _data = decltype(_data)::make(allocator, max_rigid_bodies).second;
    _data.resize(max_rigid_bodies);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_rigid_bodies).second;
    _available_handles.resize(max_rigid_bodies);
    for (auto i = std::size_t{}; i != max_rigid_bodies; ++i) {
      _available_handles[i] = {
          static_cast<std::uint32_t>(max_rigid_bodies - i - 1)};
    }
    _occupancy_bits = Bit_list::make(allocator, max_rigid_bodies).second;
    _occupancy_bits.resize(max_rigid_bodies);
  }

  Rigid_body_handle create(Rigid_body_data const &data) {
    if (_available_handles.empty()) {
      throw Capacity_error{"Capacity_error in Rigid_body_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.value].construct(data);
    _occupancy_bits.set(result.value);
    return result;
  }

  void destroy(Rigid_body_handle rigid_body) {
    _available_handles.emplace_back(rigid_body);
    _occupancy_bits.reset(rigid_body.value);
  }

  Rigid_body_data const *data(Rigid_body_handle rigid_body) const noexcept {
    return _data[rigid_body.value].get();
  }

  Rigid_body_data *data(Rigid_body_handle rigid_body) noexcept {
    return _data[rigid_body.value].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        auto const handle = Rigid_body_handle{static_cast<std::uint32_t>(i)};
        f(handle, data(handle));
        ++k;
      }
    }
  }

private:
  List<Lifetime_box<Rigid_body_data>> _data;
  List<Rigid_body_handle> _available_handles;
  Bit_list _occupancy_bits;
};

class Static_body_storage {
  using Allocator = Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<Block, Static_body_storage>
  make(Allocator &allocator, std::size_t max_static_bodies) {
    auto const block = allocator.alloc(memory_requirement(max_static_bodies));
    return {block, Static_body_storage{block, max_static_bodies}};
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_static_bodies) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_static_bodies),
        decltype(_available_handles)::memory_requirement(max_static_bodies),
        decltype(_occupancy_bits)::memory_requirement(max_static_bodies),
    });
  }

  constexpr Static_body_storage() = default;

  explicit Static_body_storage(Block block,
                               std::size_t max_static_bodies) noexcept
      : Static_body_storage{block.begin, max_static_bodies} {}

  explicit Static_body_storage(void *block,
                               std::size_t max_static_bodies) noexcept {
    auto allocator =
        Allocator{make_block(block, memory_requirement(max_static_bodies))};
    _data = decltype(_data)::make(allocator, max_static_bodies).second;
    _data.resize(max_static_bodies);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_static_bodies).second;
    _available_handles.resize(max_static_bodies);
    for (auto i = std::size_t{}; i != max_static_bodies; ++i) {
      _available_handles[i] = {
          static_cast<std::uint32_t>(max_static_bodies - i - 1)};
    }
    _occupancy_bits =
        decltype(_occupancy_bits)::make(allocator, max_static_bodies).second;
    _occupancy_bits.resize(max_static_bodies);
  }

  Static_body_handle create(Static_body_data const &data) {
    if (_available_handles.empty()) {
      throw Capacity_error{"Out of space for static rigid bodies"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.value].construct(data);
    _occupancy_bits.set(result.value);
    return result;
  }

  void destroy(Static_body_handle static_body) {
    _available_handles.emplace_back(static_body);
    _occupancy_bits.reset(static_body.value);
  }

  Static_body_data const *data(Static_body_handle static_body) const noexcept {
    return _data[static_body.value].get();
  }

  Static_body_data *data(Static_body_handle static_body) noexcept {
    return _data[static_body.value].get();
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
  List<Lifetime_box<Static_body_data>> _data;
  List<Static_body_handle> _available_handles;
  Bit_list _occupancy_bits;
};

class Dynamic_object_list {
  using Allocator = Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<Block, Dynamic_object_list> make(Allocator &allocator,
                                                    std::size_t max_size) {
    auto const block = allocator.alloc(memory_requirement(max_size));
    return {block, Dynamic_object_list{block, max_size}};
  }

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
    _object_types = List<Object_type>::make(allocator, max_size).second;
    _object_handles = List<Object_handle>::make(allocator, max_size).second;
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

class Neighbor_group_storage {
  using Allocator = Stack_allocator<>;

public:
  struct Group {
    std::uint32_t objects_begin;
    std::uint32_t objects_end;
    std::uint32_t neighbor_pairs_begin;
    std::uint32_t neighbor_pairs_end;
  };

  template <typename Allocator>
  static std::pair<Block, Neighbor_group_storage>
  make(Allocator &allocator,
       std::size_t max_object_count,
       std::size_t max_neighbor_pair_count,
       std::size_t max_group_count) {
    auto const block = allocator.alloc(memory_requirement(
        max_object_count, max_neighbor_pair_count, max_group_count));
    return {
        block,
        Neighbor_group_storage{
            block, max_object_count, max_neighbor_pair_count, max_group_count}};
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_object_count,
                     std::size_t max_neighbor_pair_count,
                     std::size_t max_group_count) {
    return Allocator::memory_requirement({
        decltype(_objects)::memory_requirement(max_object_count),
        decltype(_neighbor_pairs)::memory_requirement(max_neighbor_pair_count),
        decltype(_groups)::memory_requirement(max_group_count),
    });
  }

  constexpr Neighbor_group_storage() noexcept = default;

  Neighbor_group_storage(Block block,
                         std::size_t max_object_count,
                         std::size_t max_neighbor_pair_count,
                         std::size_t max_group_count)
      : Neighbor_group_storage{block.begin,
                               max_object_count,
                               max_neighbor_pair_count,
                               max_group_count} {}

  Neighbor_group_storage(void *block_begin,
                         std::size_t max_object_count,
                         std::size_t max_neighbor_pair_count,
                         std::size_t max_group_count) {
    auto allocator = Allocator{make_block(
        block_begin,
        memory_requirement(
            max_object_count, max_neighbor_pair_count, max_group_count))};
    _objects = decltype(_objects)::make(allocator, max_object_count).second;
    _neighbor_pairs =
        decltype(_neighbor_pairs)::make(allocator, max_neighbor_pair_count)
            .second;
    _groups = decltype(_groups)::make(allocator, max_group_count).second;
  }

  std::size_t object_count() const noexcept { return _objects.size(); }

  std::variant<Particle_handle, Rigid_body_handle>
  object(std::size_t object_index) const noexcept {
    return _objects.at(object_index);
  }

  std::size_t neighbor_pair_count() const noexcept {
    return _neighbor_pairs.size();
  }

  Neighbor_pair *neighbor_pair(std::size_t neighbor_pair_index) const noexcept {
    return _neighbor_pairs[neighbor_pair_index];
  }

  std::size_t group_count() const noexcept { return _groups.size(); }

  Group const &group(std::size_t group_index) const noexcept {
    return _groups[group_index];
  }

  void clear() noexcept {
    _objects.clear();
    _neighbor_pairs.clear();
    _groups.clear();
  }

  void begin_group() {
    auto const objects_index = static_cast<std::uint32_t>(_objects.size());
    auto const neighbor_pairs_index =
        static_cast<std::uint32_t>(_neighbor_pairs.size());
    _groups.push_back({
        .objects_begin = objects_index,
        .objects_end = objects_index,
        .neighbor_pairs_begin = neighbor_pairs_index,
        .neighbor_pairs_end = neighbor_pairs_index,
    });
  }

  void add_to_group(Particle_handle object) {
    _objects.push_back(object);
    ++_groups.back().objects_end;
  }

  void add_to_group(Rigid_body_handle object) {
    _objects.push_back(object);
    ++_groups.back().objects_end;
  }

  void add_to_group(std::variant<Particle_handle, Rigid_body_handle> object) {
    std::visit([this](auto &&arg) { add_to_group(arg); }, object);
  }

  void add_to_group(Neighbor_pair *neighbor_pair) {
    _neighbor_pairs.emplace_back(neighbor_pair);
    ++_groups.back().neighbor_pairs_end;
  }

private:
  Dynamic_object_list _objects;
  List<Neighbor_pair *> _neighbor_pairs;
  List<Group> _groups;
};

class Color_group_storage {
  using Allocator = Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<Block, Color_group_storage>
  make(Allocator &allocator, std::size_t max_neighbor_pairs) {
    auto const block = allocator.alloc(memory_requirement(max_neighbor_pairs));
    return {block, Color_group_storage{block, max_neighbor_pairs}};
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_neighbor_pairs) noexcept {
    return Allocator::memory_requirement({
        decltype(_neighbor_pairs)::memory_requirement(max_neighbor_pairs),
        decltype(_groups)::memory_requirement(max_colors),
    });
  }

  constexpr Color_group_storage() = default;

  explicit Color_group_storage(Block block, std::size_t max_neighbor_pairs)
      : Color_group_storage{block.begin, max_neighbor_pairs} {}

  explicit Color_group_storage(void *block, std::size_t max_neighbor_pairs) {
    auto allocator =
        Allocator{make_block(block, memory_requirement(max_neighbor_pairs))};
    _neighbor_pairs =
        List<Neighbor_pair *>::make(allocator, max_neighbor_pairs).second;
    _groups = List<Group>::make(allocator, max_colors).second;
    _groups.resize(max_colors);
  }

  std::span<Neighbor_pair *const> group(std::uint16_t color) const noexcept {
    return {_neighbor_pairs.data() + _groups[color].neighbor_pairs_begin,
            _neighbor_pairs.data() + _groups[color].neighbor_pairs_end};
  }

  void clear() noexcept {
    _neighbor_pairs.clear();
    _groups.clear();
    _groups.resize(max_colors);
  }

  void count(std::uint16_t color) noexcept {
    ++_groups[color].neighbor_pairs_end;
  }

  void reserve() {
    for (auto &group : _groups) {
      if (group.neighbor_pairs_end == 0) {
        return;
      }
      auto const index = static_cast<std::uint32_t>(_neighbor_pairs.size());
      _neighbor_pairs.resize(_neighbor_pairs.size() + group.neighbor_pairs_end);
      group.neighbor_pairs_begin = index;
      group.neighbor_pairs_end = index;
    }
  }

  void push_back(Neighbor_pair *neighbor_pair) {
    _neighbor_pairs[_groups[neighbor_pair->color].neighbor_pairs_end++] =
        neighbor_pair;
  }

private:
  struct Group {
    std::uint32_t neighbor_pairs_begin{};
    std::uint32_t neighbor_pairs_end{};
  };

  List<Neighbor_pair *> _neighbor_pairs;
  List<Group> _groups;
};

struct Positional_constraint_problem {
  std::array<Vec3f, 2> local_position;
  std::array<Vec3f, 2> local_direction;
  float distance;
  std::array<float, 2> inverse_mass;
  std::array<Mat3x3f, 2> inverse_inertia_tensor;
};

float solve_positional_constraint(
    Positional_constraint_problem const &problem) {
  auto const &r_1 = problem.local_position[0];
  auto const &r_2 = problem.local_position[1];
  auto const &n_1 = problem.local_direction[0];
  auto const &n_2 = problem.local_direction[1];
  auto const c = problem.distance;
  auto const m_inv_1 = problem.inverse_mass[0];
  auto const m_inv_2 = problem.inverse_mass[1];
  auto const &I_inv_1 = problem.inverse_inertia_tensor[0];
  auto const &I_inv_2 = problem.inverse_inertia_tensor[1];
  auto const r_1_cross_n = cross(r_1, n_1);
  auto const r_2_cross_n = cross(r_2, n_2);
  auto const w_1 = m_inv_1 + dot(r_1_cross_n, I_inv_1 * r_1_cross_n);
  auto const w_2 = m_inv_2 + dot(r_2_cross_n, I_inv_2 * r_2_cross_n);
  return c / (w_1 + w_2);
}

struct Contact {
  Vec3f normal;
  std::array<Vec3f, 2> local_positions;
  float separating_velocity;
  float lambda_n;
  float lambda_t;
};

struct Solve_state {
  std::latch *latch;
  Particle_storage *particles;
  Rigid_body_storage *rigid_bodies;
  Static_body_storage *static_bodies;
  float inverse_delta_time;
  float restitution_separating_velocity_threshold;
};

auto constexpr max_solve_chunk_size = std::size_t{16};

struct Solve_chunk {
  Neighbor_pair const *const *pairs;
  Contact *contacts;
  std::size_t size;
};

class Position_solve_task : public util::Task {
public:
  explicit Position_solve_task(Solve_state const *state,
                               Solve_chunk const *chunk) noexcept
      : _state{state}, _chunk{chunk} {}

  void run(unsigned) final {
    for (auto i = std::size_t{}; i != _chunk->size; ++i) {
      auto const pair = _chunk->pairs[i];
      auto contact = std::optional<Contact>{};
      switch (pair->type) {
      case Object_pair_type::particle_particle: {
        contact =
            solve_neighbor_pair({get_data(Particle_handle{pair->objects[0]}),
                                 get_data(Particle_handle{pair->objects[1]})});
        break;
      }
      case Object_pair_type::particle_rigid_body: {
        contact = solve_neighbor_pair(
            {get_data(Particle_handle{pair->objects[0]}),
             get_data(Rigid_body_handle{pair->objects[1]})});
        break;
      }
      case Object_pair_type::particle_static_body: {
        contact = solve_neighbor_pair(
            {get_data(Particle_handle{pair->objects[0]}),
             get_data(Static_body_handle{pair->objects[1]})});
        break;
      }
      case Object_pair_type::rigid_body_rigid_body: {
        contact = solve_neighbor_pair(
            {get_data(Rigid_body_handle{pair->objects[0]}),
             get_data(Rigid_body_handle{pair->objects[1]})});
        break;
      }
      case Object_pair_type::rigid_body_static_body: {
        contact = solve_neighbor_pair(
            {get_data(Rigid_body_handle{pair->objects[0]}),
             get_data(Static_body_handle{pair->objects[1]})});
        break;
      }
      default:
        math::unreachable();
      }
      if (contact) {
        _chunk->contacts[i] = *contact;
      } else {
        _chunk->contacts[i].normal = Vec3f::zero();
      }
    }
    _state->latch->count_down();
  }

private:
  std::optional<Contact> solve_neighbor_pair(
      std::pair<Particle_data *, Particle_data *> data) const noexcept {
    if (auto const contact_geometry = get_contact_geometry(data)) {
      auto contact = Contact{
          .normal = contact_geometry->normal,
          .separating_velocity =
              dot(get_velocity(data.first) - get_velocity(data.second),
                  contact_geometry->normal),
          .lambda_n = 0.0f,
          .lambda_t = 0.0f,
      };
      solve_contact(data, contact, contact_geometry->separation);
      return contact;
    } else {
      return std::nullopt;
    }
  }

  std::optional<Contact> solve_neighbor_pair(
      std::pair<Particle_data *, Rigid_body_data *> data) const noexcept {
    if (auto const contact_geometry = get_contact_geometry(data)) {
      auto const relative_position =
          contact_geometry->position - get_position(data.second);
      auto const local_position =
          transpose(get_rotation(data.second)) * relative_position;
      auto contact = Contact{
          .normal = contact_geometry->normal,
          .local_positions = {Vec3f{}, local_position},
          .separating_velocity =
              dot(get_velocity(data.first) -
                      get_velocity(data.second, relative_position),
                  contact_geometry->normal),
          .lambda_n = 0.0f,
          .lambda_t = 0.0f,
      };
      solve_contact(data, contact, contact_geometry->separation);
      return contact;
    } else {
      return std::nullopt;
    }
  }

  std::optional<Contact> solve_neighbor_pair(
      std::pair<Particle_data *, Static_body_data *> data) const noexcept {
    if (auto const contact_geometry = get_contact_geometry(data)) {
      auto contact = Contact{
          .normal = contact_geometry->normal,
          .separating_velocity =
              dot(get_velocity(data.first), contact_geometry->normal),
          .lambda_n = 0.0f,
          .lambda_t = 0.0f,
      };
      solve_contact(data, contact, contact_geometry->separation);
      return contact;
    } else {
      return std::nullopt;
    }
  }

  std::optional<Contact> solve_neighbor_pair(
      std::pair<Rigid_body_data *, Rigid_body_data *> data) const noexcept {
    if (auto const contact_geometry = get_contact_geometry(data)) {
      auto const relative_positions = std::array<Vec3f, 2>{
          contact_geometry->position - get_position(data.first),
          contact_geometry->position - get_position(data.second),
      };
      auto const local_positions = std::array<Vec3f, 2>{
          transpose(get_rotation(data.first)) * relative_positions[0],
          transpose(get_rotation(data.second)) * relative_positions[1],
      };
      auto contact = Contact{
          .normal = contact_geometry->normal,
          .local_positions = local_positions,
          .separating_velocity =
              dot(get_velocity(data.first, relative_positions[0]) -
                      get_velocity(data.second, relative_positions[1]),
                  contact_geometry->normal),
          .lambda_n = 0.0f,
          .lambda_t = 0.0f,
      };
      solve_contact(data, contact, contact_geometry->separation);
      return contact;
    } else {
      return std::nullopt;
    }
  }

  std::optional<Contact> solve_neighbor_pair(
      std::pair<Rigid_body_data *, Static_body_data *> data) const noexcept {
    if (auto const contact_geometry = get_contact_geometry(data)) {
      auto const relative_position =
          contact_geometry->position - get_position(data.first);
      auto const local_position =
          transpose(get_rotation(data.first)) * relative_position;
      auto contact = Contact{
          .normal = contact_geometry->normal,
          .local_positions = {local_position, Vec3f{}},
          .separating_velocity =
              dot(get_velocity(data.first, relative_position),
                  contact_geometry->normal),
          .lambda_n = 0.0f,
          .lambda_t = 0.0f,
      };
      solve_contact(data, contact, contact_geometry->separation);
      return contact;
    } else {
      return std::nullopt;
    }
  }

  std::optional<Positionless_contact_geometry> get_contact_geometry(
      std::pair<Particle_data *, Particle_data *> data) const noexcept {
    auto const displacement = data.first->position - data.second->position;
    auto const distance2 = length_squared(displacement);
    auto const contact_distance = data.first->radius + data.second->radius;
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
      return Positionless_contact_geometry{
          .normal = normal,
          .separation = separation,
      };
    } else {
      return std::nullopt;
    }
  }

  std::optional<Positionful_contact_geometry> get_contact_geometry(
      std::pair<Particle_data *, Rigid_body_data *> data) const noexcept {
    auto const transform =
        Mat3x4f::rigid(data.second->position, data.second->orientation);
    auto const inverse_transform = rigid_inverse(transform);
    return particle_shape_positionful_contact_geometry(data.first->position,
                                                       data.first->radius,
                                                       data.second->shape,
                                                       transform,
                                                       inverse_transform);
  }

  std::optional<Positionless_contact_geometry> get_contact_geometry(
      std::pair<Particle_data *, Static_body_data *> data) const noexcept {
    return particle_shape_positionless_contact_geometry(
        data.first->position,
        data.first->radius,
        data.second->shape,
        data.second->transform,
        data.second->inverse_transform);
  }

  std::optional<Positionful_contact_geometry> get_contact_geometry(
      std::pair<Rigid_body_data *, Rigid_body_data *> data) const noexcept {
    auto const transforms = std::array<Mat3x4f, 2>{
        Mat3x4f::rigid(data.first->position, data.first->orientation),
        Mat3x4f::rigid(data.second->position, data.second->orientation),
    };
    auto const inverse_transforms = std::array<Mat3x4f, 2>{
        rigid_inverse(transforms[0]),
        rigid_inverse(transforms[1]),
    };
    return shape_shape_contact_geometry(data.first->shape,
                                        transforms[0],
                                        inverse_transforms[0],
                                        data.second->shape,
                                        transforms[1],
                                        inverse_transforms[1]);
  }

  std::optional<Positionful_contact_geometry> get_contact_geometry(
      std::pair<Rigid_body_data *, Static_body_data *> data) const noexcept {
    auto const transform =
        Mat3x4f::rigid(data.first->position, data.first->orientation);
    auto const inverse_transform = rigid_inverse(transform);
    return shape_shape_contact_geometry(data.first->shape,
                                        transform,
                                        inverse_transform,
                                        data.second->shape,
                                        data.second->transform,
                                        data.second->inverse_transform);
  }

  template <typename T, typename U>
  void solve_contact(std::pair<T, U> data,
                     Contact &contact,
                     float separation) const noexcept {
    auto const rotation = std::array<Mat3x3f, 2>{
        get_rotation(data.first),
        get_rotation(data.second),
    };
    auto const inverse_rotation = std::array<Mat3x3f, 2>{
        transpose(rotation[0]),
        transpose(rotation[1]),
    };
    auto const local_normal = std::array<Vec3f, 2>{
        inverse_rotation[0] * contact.normal,
        inverse_rotation[1] * contact.normal,
    };
    auto const inverse_mass = std::array<float, 2>{
        get_inverse_mass(data.first),
        get_inverse_mass(data.second),
    };
    auto const inverse_inertia_tensor = std::array<Mat3x3f, 2>{
        get_inverse_inertia_tensor(data.first),
        get_inverse_inertia_tensor(data.second),
    };
    contact.lambda_n = solve_positional_constraint({
        .local_position = contact.local_positions,
        .local_direction = local_normal,
        .distance = -separation,
        .inverse_mass = inverse_mass,
        .inverse_inertia_tensor = inverse_inertia_tensor,
    });
    auto global_impulse = contact.lambda_n * contact.normal;
    auto local_impulse = std::array<Vec3f, 2>{
        contact.lambda_n * local_normal[0],
        contact.lambda_n * local_normal[1],
    };
    apply_impulse(data.first,
                  rotation[0] * inverse_inertia_tensor[0],
                  contact.local_positions[0],
                  local_impulse[0],
                  global_impulse);
    apply_impulse(data.second,
                  rotation[1] * inverse_inertia_tensor[1],
                  contact.local_positions[1],
                  -local_impulse[1],
                  -global_impulse);
    auto const current_contact_position = std::array<Vec3f, 2>{
        get_position(data.first) + rotation[0] * contact.local_positions[0],
        get_position(data.second) + rotation[1] * contact.local_positions[1],
    };
    auto const previous_contact_position = std::array<Vec3f, 2>{
        get_previous_position(data.first) +
            get_previous_rotation(data.first) * contact.local_positions[0],
        get_previous_position(data.second) +
            get_previous_rotation(data.second) * contact.local_positions[1],
    };
    auto const relative_contact_motion =
        (current_contact_position[0] - previous_contact_position[0]) -
        (current_contact_position[1] - previous_contact_position[1]);
    auto const tangential_relative_contact_motion =
        perp_unit(relative_contact_motion, contact.normal);
    if (tangential_relative_contact_motion != Vec3f::zero()) {
      auto const correction_distance =
          length(tangential_relative_contact_motion);
      auto const correction_direction =
          tangential_relative_contact_motion / -correction_distance;
      auto const local_correction_direction = std::array<Vec3f, 2>{
          inverse_rotation[0] * correction_direction,
          inverse_rotation[1] * correction_direction,
      };
      auto const lambda_t = solve_positional_constraint({
          .local_position = contact.local_positions,
          .local_direction = local_correction_direction,
          .distance = correction_distance,
          .inverse_mass = inverse_mass,
          .inverse_inertia_tensor = inverse_inertia_tensor,
      });
      auto const static_friction_coefficient =
          0.5f * (data.first->material.static_friction_coefficient +
                  data.second->material.static_friction_coefficient);
      if (lambda_t < static_friction_coefficient * contact.lambda_n) {
        contact.lambda_t = lambda_t;
        global_impulse = contact.lambda_t * correction_direction;
        local_impulse[0] = contact.lambda_t * local_correction_direction[0];
        local_impulse[1] = contact.lambda_t * local_correction_direction[1];
        apply_impulse(data.first,
                      rotation[0] * inverse_inertia_tensor[0],
                      contact.local_positions[0],
                      local_impulse[0],
                      global_impulse);
        apply_impulse(data.second,
                      rotation[1] * inverse_inertia_tensor[1],
                      contact.local_positions[1],
                      -local_impulse[1],
                      -global_impulse);
      }
    }
  }

  void apply_impulse(Particle_data *particle,
                     Mat3x3f const &,
                     Vec3f const &,
                     Vec3f const &,
                     Vec3f const &global_impulse) const noexcept {
    particle->position += global_impulse * particle->inverse_mass;
  }

  void apply_impulse(Rigid_body_data *rigid_body,
                     Mat3x3f const &rotated_inverse_inertia_tensor,
                     Vec3f const &local_position,
                     Vec3f const &local_impulse,
                     Vec3f const &global_impulse) const noexcept {
    rigid_body->position += global_impulse * rigid_body->inverse_mass;
    rigid_body->orientation += 0.5f *
                               Quatf{0.0f,
                                     rotated_inverse_inertia_tensor *
                                         cross(local_position, local_impulse)} *
                               rigid_body->orientation;
  }

  void apply_impulse(Static_body_data *,
                     Mat3x3f const &,
                     Vec3f const &,
                     Vec3f const &,
                     Vec3f const &) const noexcept {}

  Vec3f get_previous_position(Particle_data *particle) const noexcept {
    return particle->previous_position;
  }

  Vec3f get_previous_position(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->previous_position;
  }

  Vec3f get_previous_position(Static_body_data *static_body) const noexcept {
    return column(static_body->transform, 3);
  }

  Vec3f get_position(Particle_data *particle) const noexcept {
    return particle->position;
  }

  Vec3f get_position(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->position;
  }

  Vec3f get_position(Static_body_data *static_body) const noexcept {
    return column(static_body->transform, 3);
  }

  Mat3x3f get_previous_rotation(Particle_data *particle) const noexcept {
    return get_rotation(particle);
  }

  Mat3x3f get_previous_rotation(Rigid_body_data *rigid_body) const noexcept {
    return Mat3x3f::rotation(rigid_body->previous_orientation);
  }

  Mat3x3f get_previous_rotation(Static_body_data *static_body) const noexcept {
    return get_rotation(static_body);
  }

  Mat3x3f get_rotation(Particle_data *) const noexcept {
    return Mat3x3f::identity();
  }

  Mat3x3f get_rotation(Rigid_body_data *rigid_body) const noexcept {
    return Mat3x3f::rotation(rigid_body->orientation);
  }

  Mat3x3f get_rotation(Static_body_data *static_body) const noexcept {
    return {{static_body->transform[0][0],
             static_body->transform[0][1],
             static_body->transform[0][2]},
            {static_body->transform[1][0],
             static_body->transform[1][1],
             static_body->transform[1][2]},
            {static_body->transform[2][0],
             static_body->transform[2][1],
             static_body->transform[2][2]}};
  }

  Vec3f get_velocity(Particle_data *particle) const noexcept {
    return particle->velocity;
  }

  Vec3f get_velocity(Rigid_body_data *rigid_body,
                     Vec3f const &relative_position) const noexcept {
    return rigid_body->velocity +
           cross(rigid_body->angular_velocity, relative_position);
  }

  float get_inverse_mass(Particle_data *particle) const noexcept {
    return particle->inverse_mass;
  }

  float get_inverse_mass(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->inverse_mass;
  }

  float get_inverse_mass(Static_body_data *) const noexcept { return 0.0f; }

  Mat3x3f get_inverse_inertia_tensor(Particle_data *) const noexcept {
    return Mat3x3f::zero();
  }

  Mat3x3f
  get_inverse_inertia_tensor(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->inverse_inertia_tensor;
  }

  Mat3x3f get_inverse_inertia_tensor(Static_body_data *) const noexcept {
    return Mat3x3f::zero();
  }

  Particle_data *get_data(Particle_handle particle) const noexcept {
    return _state->particles->data(particle);
  }

  Rigid_body_data *get_data(Rigid_body_handle rigid_body) const noexcept {
    return _state->rigid_bodies->data(rigid_body);
  }

  Static_body_data *get_data(Static_body_handle static_body) const noexcept {
    return _state->static_bodies->data(static_body);
  }

  Solve_state const *_state;
  Solve_chunk const *_chunk;
};

class Velocity_solve_task : public util::Task {
public:
  explicit Velocity_solve_task(Solve_state const *state,
                               Solve_chunk const *chunk) noexcept
      : _state{state}, _chunk{chunk} {}

  void run(unsigned) final {
    for (auto i = std::size_t{}; i != _chunk->size; ++i) {
      auto const &contact = _chunk->contacts[i];
      auto const &normal = contact.normal;
      if (normal != math::Vec3f::zero()) {
        switch (_chunk->pairs[i]->type) {
        case Object_pair_type::particle_particle:
          solve_contact(
              std::pair{Particle_handle{_chunk->pairs[i]->objects[0]},
                        Particle_handle{_chunk->pairs[i]->objects[1]}},
              contact);
          continue;
        case Object_pair_type::particle_rigid_body:
          solve_contact(
              std::pair{Particle_handle{_chunk->pairs[i]->objects[0]},
                        Rigid_body_handle{_chunk->pairs[i]->objects[1]}},
              contact);
          continue;
        case Object_pair_type::particle_static_body:
          solve_contact(
              std::pair{Particle_handle{_chunk->pairs[i]->objects[0]},
                        Static_body_handle{_chunk->pairs[i]->objects[1]}},
              contact);
          continue;
        case Object_pair_type::rigid_body_rigid_body:
          solve_contact(
              std::pair{Rigid_body_handle{_chunk->pairs[i]->objects[0]},
                        Rigid_body_handle{_chunk->pairs[i]->objects[1]}},
              contact);
          continue;
        case Object_pair_type::rigid_body_static_body:
          solve_contact(
              std::pair{Rigid_body_handle{_chunk->pairs[i]->objects[0]},
                        Static_body_handle{_chunk->pairs[i]->objects[1]}},
              contact);
          continue;
        }
      }
    }
    _state->latch->count_down();
  }

private:
  template <typename T, typename U>
  void solve_contact(std::pair<T, U> objects,
                     Contact const &contact) const noexcept {
    auto const data =
        std::pair{get_data(objects.first), get_data(objects.second)};
    auto const inverse_inertia_tensor = std::array<Mat3x3f, 2>{
        get_inverse_inertia_tensor(data.first),
        get_inverse_inertia_tensor(data.second),
    };
    auto const rotation = std::array<Mat3x3f, 2>{
        get_rotation(data.first),
        get_rotation(data.second),
    };
    auto const relative_position = std::array<Vec3f, 2>{
        rotation[0] * contact.local_positions[0],
        rotation[1] * contact.local_positions[1],
    };
    apply_restitution(data, contact, inverse_inertia_tensor, relative_position);
    if (!std::is_same_v<T, U> && contact.lambda_t == 0.0f) {
      apply_dynamic_friction(
          data, contact, inverse_inertia_tensor, relative_position);
    }
  }

  template <typename T, typename U>
  void apply_restitution(
      std::pair<T, U> data,
      Contact const &contact,
      std::array<Mat3x3f, 2> const &inverse_inertia_tensor,
      std::array<Vec3f, 2> const &relative_position) const noexcept {
    auto const relative_velocity =
        get_velocity(data.first, relative_position[0]) -
        get_velocity(data.second, relative_position[1]);
    auto const separating_velocity = dot(contact.normal, relative_velocity);
    auto const delta_velocity =
        get_restitution_velocity_update(data, contact, separating_velocity);
    if (delta_velocity != Vec3f::zero()) {
      auto const delta_velocity_direction = normalize(delta_velocity);
      auto const w_1 = get_generalized_inverse_mass(data.first,
                                                    inverse_inertia_tensor[0],
                                                    relative_position[0],
                                                    delta_velocity_direction);
      auto const w_2 = get_generalized_inverse_mass(data.second,
                                                    inverse_inertia_tensor[0],
                                                    relative_position[1],
                                                    delta_velocity_direction);
      auto const impulse = delta_velocity / (w_1 + w_2);
      apply_impulse(
          data.first, inverse_inertia_tensor[0], relative_position[0], impulse);
      apply_impulse(data.second,
                    inverse_inertia_tensor[1],
                    relative_position[1],
                    -impulse);
    }
  }

  template <typename T, typename U>
  Vec3f
  get_restitution_velocity_update(std::pair<T, U> data,
                                  Contact const &contact,
                                  float separating_velocity) const noexcept {
    auto const restitution_coefficient = [&] {
      if (abs(separating_velocity) >
          _state->restitution_separating_velocity_threshold) {
        return 0.5f * (get_restitution_coefficient(data.first) +
                       get_restitution_coefficient(data.second));
      } else {
        return 0.0f;
      }
    }();
    return contact.normal *
           (-separating_velocity +
            min(-restitution_coefficient * contact.separating_velocity, 0.0f));
  }

  template <typename T, typename U>
  void apply_dynamic_friction(
      std::pair<T, U> data,
      Contact const &contact,
      std::array<Mat3x3f, 2> const &inverse_inertia_tensor,
      std::array<Vec3f, 2> const &relative_position) const noexcept {
    auto const relative_velocity =
        get_velocity(data.first, relative_position[0]) -
        get_velocity(data.second, relative_position[1]);
    auto const separating_velocity = dot(contact.normal, relative_velocity);
    auto const tangential_velocity =
        relative_velocity - contact.normal * separating_velocity;
    auto const delta_velocity =
        get_friction_velocity_update(data, contact, tangential_velocity);
    if (delta_velocity != Vec3f::zero()) {
      auto const delta_velocity_direction = normalize(delta_velocity);
      auto const w_1 = get_generalized_inverse_mass(data.first,
                                                    inverse_inertia_tensor[0],
                                                    relative_position[0],
                                                    delta_velocity_direction);
      auto const w_2 = get_generalized_inverse_mass(data.second,
                                                    inverse_inertia_tensor[1],
                                                    relative_position[1],
                                                    delta_velocity_direction);
      auto const impulse = delta_velocity / (w_1 + w_2);
      apply_impulse(
          data.first, inverse_inertia_tensor[0], relative_position[0], impulse);
      apply_impulse(data.second,
                    inverse_inertia_tensor[1],
                    relative_position[1],
                    -impulse);
    }
  }

  template <typename T, typename U>
  Vec3f get_friction_velocity_update(
      std::pair<T, U> data,
      Contact const &contact,
      Vec3f const &tangential_velocity) const noexcept {
    if (tangential_velocity != Vec3f::zero()) {
      auto const dynamic_friction_coefficient =
          0.5f * (get_dynamic_friction_coefficient(data.first) +
                  get_dynamic_friction_coefficient(data.second));
      auto const tangential_speed = length(tangential_velocity);
      auto const delta_velocity_direction =
          -tangential_velocity / tangential_speed;
      return delta_velocity_direction *
             min(dynamic_friction_coefficient * contact.lambda_n *
                     _state->inverse_delta_time,
                 tangential_speed);
    } else {
      return Vec3f::zero();
    }
  }

  void apply_impulse(Particle_data *particle,
                     Mat3x3f const &,
                     Vec3f const &,
                     Vec3f const &impulse) const noexcept {
    particle->velocity += impulse * particle->inverse_mass;
  }

  void apply_impulse(Rigid_body_data *rigid_body,
                     Mat3x3f const &inverse_inertia_tensor,
                     Vec3f const &relative_position,
                     Vec3f const &impulse) const noexcept {
    rigid_body->velocity += impulse * rigid_body->inverse_mass;
    rigid_body->angular_velocity +=
        inverse_inertia_tensor * cross(relative_position, impulse);
  }

  void apply_impulse(Static_body_data *,
                     Mat3x3f const &,
                     Vec3f const &,
                     Vec3f const &) const noexcept {}

  Mat3x3f get_rotation(Particle_data *) const noexcept {
    return Mat3x3f::identity();
  }

  Mat3x3f get_rotation(Rigid_body_data *rigid_body) const noexcept {
    return Mat3x3f::rotation(rigid_body->orientation);
  }

  Mat3x3f get_rotation(Static_body_data *static_body) const noexcept {
    return {{static_body->transform[0][0],
             static_body->transform[0][1],
             static_body->transform[0][2]},
            {static_body->transform[1][0],
             static_body->transform[1][1],
             static_body->transform[1][2]},
            {static_body->transform[2][0],
             static_body->transform[2][1],
             static_body->transform[2][2]}};
  }

  Vec3f get_velocity(Particle_data *particle, Vec3f const &) const noexcept {
    return particle->velocity;
  }

  Vec3f get_velocity(Rigid_body_data *rigid_body,
                     Vec3f const &relative_position) const noexcept {
    return rigid_body->velocity +
           cross(rigid_body->angular_velocity, relative_position);
  }

  Vec3f get_velocity(Static_body_data *, Vec3f const &) const noexcept {
    return Vec3f::zero();
  }

  Mat3x3f get_inverse_inertia_tensor(Particle_data *) const noexcept {
    return Mat3x3f::zero();
  }

  Mat3x3f
  get_inverse_inertia_tensor(Rigid_body_data *rigid_body) const noexcept {
    auto const rotation = Mat3x3f::rotation(rigid_body->orientation);
    return rotation * rigid_body->inverse_inertia_tensor * transpose(rotation);
  }

  Mat3x3f get_inverse_inertia_tensor(Static_body_data *) const noexcept {
    return Mat3x3f::zero();
  }

  float get_generalized_inverse_mass(Particle_data *particle,
                                     Mat3x3f const &,
                                     Vec3f const &,
                                     Vec3f const &) const noexcept {
    return particle->inverse_mass;
  }

  float get_generalized_inverse_mass(Rigid_body_data *rigid_body,
                                     Mat3x3f const &inverse_inertia_tensor,
                                     Vec3f const &relative_position,
                                     Vec3f const &direction) const noexcept {
    auto const r_cross_n = cross(relative_position, direction);
    return rigid_body->inverse_mass +
           dot(r_cross_n, inverse_inertia_tensor * r_cross_n);
  }

  float get_generalized_inverse_mass(Static_body_data *,
                                     Mat3x3f const &,
                                     Vec3f const &,
                                     Vec3f const &) const noexcept {
    return 0.0f;
  }

  // float
  // get_static_friction_coefficient(Particle_data *particle) const noexcept {
  //   return particle->material.static_friction_coefficient;
  // }

  // float
  // get_static_friction_coefficient(Rigid_body_data *rigid_body) const
  // noexcept
  // {
  //   return rigid_body->material.static_friction_coefficient;
  // }

  // float get_static_friction_coefficient(
  //     Static_body_data *static_body) const noexcept {
  //   return static_body->material.static_friction_coefficient;
  // }

  float
  get_dynamic_friction_coefficient(Particle_data *particle) const noexcept {
    return particle->material.dynamic_friction_coefficient;
  }

  float
  get_dynamic_friction_coefficient(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->material.dynamic_friction_coefficient;
  }

  float get_dynamic_friction_coefficient(
      Static_body_data *static_body) const noexcept {
    return static_body->material.dynamic_friction_coefficient;
  }

  float get_restitution_coefficient(Particle_data *particle) const noexcept {
    return particle->material.restitution_coefficient;
  }

  float
  get_restitution_coefficient(Rigid_body_data *rigid_body) const noexcept {
    return rigid_body->material.restitution_coefficient;
  }

  float
  get_restitution_coefficient(Static_body_data *static_body) const noexcept {
    return static_body->material.restitution_coefficient;
  }

  Particle_data *get_data(Particle_handle particle) const noexcept {
    return _state->particles->data(particle);
  }

  Rigid_body_data *get_data(Rigid_body_handle rigid_body) const noexcept {
    return _state->rigid_bodies->data(rigid_body);
  }

  Static_body_data *get_data(Static_body_handle static_body) const noexcept {
    return _state->static_bodies->data(static_body);
  }

  Solve_state const *_state;
  Solve_chunk const *_chunk;
};

// integration constants
auto constexpr velocity_damping_factor = 0.99f;
auto constexpr waking_motion_epsilon = 0.01f;
auto constexpr waking_motion_initializer = 2.0f * waking_motion_epsilon;
auto constexpr waking_motion_limit = 10.0f * waking_motion_epsilon;
auto constexpr waking_motion_smoothing_factor = 0.8f;
} // namespace

class World::Impl {
public:
  friend class World;

  static constexpr std::size_t
  memory_requirement(World_create_info const &create_info) {
    return Stack_allocator<>::memory_requirement({
        decltype(_particles)::memory_requirement(create_info.max_particles),
        decltype(_rigid_bodies)::memory_requirement(
            create_info.max_rigid_bodies),
        decltype(_static_bodies)::memory_requirement(
            create_info.max_static_bodies),
        decltype(_aabb_tree)::memory_requirement(
            create_info.max_aabb_tree_leaf_nodes,
            create_info.max_aabb_tree_internal_nodes),
        decltype(_neighbor_pairs)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_neighbor_pair_ptrs)::memory_requirement(
            2 * create_info.max_neighbor_pairs),
        decltype(_neighbor_groups)::memory_requirement(
            create_info.max_particles + create_info.max_rigid_bodies,
            create_info.max_neighbor_pairs,
            create_info.max_neighbor_groups),
        decltype(_neighbor_group_awake_indices)::memory_requirement(
            create_info.max_neighbor_groups),
        decltype(_coloring_bits)::memory_requirement(max_colors),
        decltype(_coloring_fringe)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_color_groups)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_contacts)::memory_requirement(create_info.max_neighbor_pairs),
        decltype(_solve_chunks)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_position_solve_tasks)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_velocity_solve_tasks)::memory_requirement(
            create_info.max_neighbor_pairs),
    });
  }

  explicit Impl(World_create_info const &create_info)
      : _gravitational_acceleration{create_info.gravitational_acceleration} {
    _block = util::System_allocator::instance()->alloc(
        memory_requirement(create_info));
    auto allocator = Stack_allocator<>{_block};
    _particles =
        Particle_storage::make(allocator, create_info.max_particles).second;
    _rigid_bodies =
        Rigid_body_storage::make(allocator, create_info.max_rigid_bodies)
            .second;
    _static_bodies =
        Static_body_storage::make(allocator, create_info.max_static_bodies)
            .second;
    _aabb_tree = make_aabb_tree<Aabb_tree_payload_t>(
                     allocator,
                     create_info.max_aabb_tree_leaf_nodes,
                     create_info.max_aabb_tree_internal_nodes)
                     .second;
    _neighbor_pairs =
        List<Neighbor_pair>::make(allocator, create_info.max_neighbor_pairs)
            .second;
    _neighbor_pair_ptrs = List<Neighbor_pair *>::make(
                              allocator, 2 * create_info.max_neighbor_pairs)
                              .second;
    _neighbor_groups =
        Neighbor_group_storage::make(allocator,
                                     create_info.max_particles +
                                         create_info.max_rigid_bodies,
                                     create_info.max_neighbor_pairs,
                                     create_info.max_neighbor_groups)
            .second;
    _neighbor_group_awake_indices =
        List<std::uint32_t>::make(allocator, create_info.max_neighbor_groups)
            .second;
    _coloring_bits = Bit_list::make(allocator, max_colors).second;
    _coloring_bits.resize(max_colors);
    _coloring_fringe =
        Queue<Neighbor_pair *>::make(allocator, create_info.max_neighbor_pairs)
            .second;
    _color_groups =
        Color_group_storage::make(allocator, create_info.max_neighbor_pairs)
            .second;
    _contacts =
        List<Contact>::make(allocator, create_info.max_neighbor_pairs).second;
    _solve_chunks =
        List<Solve_chunk>::make(allocator, create_info.max_neighbor_pairs)
            .second;
    _position_solve_tasks = List<Position_solve_task>::make(
                                allocator, create_info.max_neighbor_pairs)
                                .second;
    _velocity_solve_tasks = List<Velocity_solve_task>::make(
                                allocator, create_info.max_neighbor_pairs)
                                .second;
  }

  ~Impl() {
    _velocity_solve_tasks = {};
    _position_solve_tasks = {};
    _solve_chunks = {};
    _contacts = {};
    _color_groups = {};
    _coloring_fringe = {};
    _coloring_bits = {};
    _neighbor_group_awake_indices = {};
    _neighbor_groups = {};
    _neighbor_pair_ptrs = {};
    _neighbor_pairs = {};
    _aabb_tree = {};
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
  create_static_body(Static_body_create_info const &create_info) {
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

  void destroy_static_body(Static_body_handle handle) {
    _aabb_tree.destroy_leaf(_static_bodies.data(handle)->aabb_tree_node);
    _static_bodies.destroy(handle);
  }

  void simulate(World const &world, World_simulate_info const &simulate_info) {
    build_aabb_tree(simulate_info.delta_time);
    clear_neighbor_pairs();
    find_neighbor_pairs();
    assign_neighbor_pairs();
    find_neighbor_groups();
    _neighbor_group_awake_indices.clear();
    _color_groups.clear();
    for (auto j = std::size_t{}; j != _neighbor_groups.group_count(); ++j) {
      if (update_neighbor_group_awake_states(j)) {
        _neighbor_group_awake_indices.emplace_back(j);
        color_neighbor_group(j);
      }
    }
    _color_groups.reserve();
    assign_color_groups();
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const h_inv = 1.0f / h;
    auto solve_state = Solve_state{
        nullptr,
        &_particles,
        &_rigid_bodies,
        &_static_bodies,
        h_inv,
        2.0f * length(_gravitational_acceleration) * h,
    };
    _contacts.clear();
    _solve_chunks.clear();
    _position_solve_tasks.clear();
    _velocity_solve_tasks.clear();
    for (auto i = std::size_t{}; i != max_colors; ++i) {
      auto const color = static_cast<std::uint16_t>(i);
      auto const group = _color_groups.group(color);
      if (!group.empty()) {
        for (auto j = std::size_t{}; j < group.size();
             j += max_solve_chunk_size) {
          auto const chunk_size = min(group.size() - j, max_solve_chunk_size);
          _solve_chunks.push_back(
              {.pairs = group.data() + j,
               .contacts = _contacts.data() + _contacts.size(),
               .size = chunk_size});
          _contacts.resize(_contacts.size() + chunk_size);
          _position_solve_tasks.emplace_back(&solve_state,
                                             &_solve_chunks.back());
          _velocity_solve_tasks.emplace_back(&solve_state,
                                             &_solve_chunks.back());
        }
      } else {
        break;
      }
    }
    auto const time_compensated_velocity_damping_factor =
        pow(velocity_damping_factor, h);
    auto const time_compensating_waking_motion_smoothing_factor =
        1.0f - pow(1.0f - waking_motion_smoothing_factor, h);
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate(h,
                time_compensated_velocity_damping_factor,
                time_compensating_waking_motion_smoothing_factor);
      // find_contacts();
      solve_positions(*simulate_info.thread_pool, solve_state);
      derive_velocities(h_inv);
      solve_velocities(*simulate_info.thread_pool, solve_state);
    }
    call_particle_motion_callbacks(world);
    call_dynamic_rigid_body_motion_callbacks(world);
  }

private:
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

  void clear_neighbor_pairs() {
    auto const reset_neighbor_count = [](auto const, auto const data) {
      data->neighbor_count = 0;
    };
    _particles.for_each(reset_neighbor_count);
    _rigid_bodies.for_each(reset_neighbor_count);
    _neighbor_pair_ptrs.clear();
    _neighbor_pairs.clear();
    _neighbor_groups.clear();
  }

  void find_neighbor_pairs() {
    _aabb_tree.for_each_overlapping_leaf_pair(
        [this](Aabb_tree_payload_t const &first_payload,
               Aabb_tree_payload_t const &second_payload) {
          std::visit(
              [&](auto &&first_handle) {
                std::visit(
                    [&](auto &&second_handle) {
                      using T = std::decay_t<decltype(first_handle)>;
                      using U = std::decay_t<decltype(second_handle)>;
                      if constexpr (!std::is_same_v<T, Static_body_handle> ||
                                    !std::is_same_v<U, Static_body_handle>) {
                        _neighbor_pairs.emplace_back(
                            std::pair{first_handle, second_handle});
                        increment_neighbor_count(get_data(first_handle));
                        increment_neighbor_count(get_data(second_handle));
                      }
                    },
                    second_payload);
              },
              first_payload);
        });
  }

  void increment_neighbor_count(Particle_data *data) noexcept {
    ++data->neighbor_count;
  }

  void increment_neighbor_count(Rigid_body_data *data) noexcept {
    ++data->neighbor_count;
  }

  void increment_neighbor_count(Static_body_data *) const noexcept {}

  void assign_neighbor_pairs() {
    auto const alloc_neighbor_pairs = [this](auto const, auto const data) {
      data->neighbor_pairs = _neighbor_pair_ptrs.end();
      _neighbor_pair_ptrs.resize(_neighbor_pair_ptrs.size() +
                                 data->neighbor_count);
      data->neighbor_count = 0;
    };
    _particles.for_each(alloc_neighbor_pairs);
    _rigid_bodies.for_each(alloc_neighbor_pairs);
    for (auto &pair : _neighbor_pairs) {
      std::visit(
          [&](auto &&handle) { assign_neighbor_pair(get_data(handle), &pair); },
          pair.first());
      std::visit(
          [&](auto &&handle) { assign_neighbor_pair(get_data(handle), &pair); },
          pair.second());
    }
  }

  void assign_neighbor_pair(Particle_data *data,
                            Neighbor_pair *neighbor_pair) noexcept {
    data->neighbor_pairs[data->neighbor_count++] = neighbor_pair;
  }

  void assign_neighbor_pair(Rigid_body_data *data,
                            Neighbor_pair *neighbor_pair) noexcept {
    data->neighbor_pairs[data->neighbor_count++] = neighbor_pair;
  }

  void assign_neighbor_pair(Static_body_data *,
                            Neighbor_pair *) const noexcept {}

  void find_neighbor_groups() {
    auto const unmark = [](auto const, auto const data) {
      data->marked = false;
    };
    _particles.for_each(unmark);
    _rigid_bodies.for_each(unmark);
    auto const visitor = [this](auto &&handle) {
      for (auto const pair : get_neighbor_pairs(get_data(handle))) {
        std::visit(
            [&](auto &&neighbor_handle) {
              using T = std::decay_t<decltype(neighbor_handle)>;
              if constexpr (!std::is_same_v<T, Static_body_handle>) {
                auto const neighbor_data = get_data(neighbor_handle);
                if (!neighbor_data->marked) {
                  neighbor_data->marked = true;
                  _neighbor_groups.add_to_group(neighbor_handle);
                }
                if (pair->color == color_unmarked) {
                  pair->color = color_marked;
                  _neighbor_groups.add_to_group(pair);
                }
              } else {
                _neighbor_groups.add_to_group(pair);
              }
            },
            pair->other(handle));
      }
    };
    auto fringe_index = std::size_t{};
    auto const find_neighbor_group = [&, this](auto const seed_handle,
                                               auto const seed_data) {
      if (!seed_data->marked) {
        seed_data->marked = true;
        _neighbor_groups.begin_group();
        _neighbor_groups.add_to_group(seed_handle);
        do {
          std::visit(visitor, _neighbor_groups.object(fringe_index));
        } while (++fringe_index != _neighbor_groups.object_count());
      }
    };
    _particles.for_each(find_neighbor_group);
    _rigid_bodies.for_each(find_neighbor_group);
  }

  bool update_neighbor_group_awake_states(std::size_t group_index) {
    auto const &group = _neighbor_groups.group(group_index);
    auto contains_awake = false;
    auto contains_sleeping = false;
    auto sleepable = true;
    for (auto i = group.objects_begin;
         (sleepable || !contains_awake || !contains_sleeping) &&
         i != group.objects_end;
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
        for (auto i = group.objects_begin; i != group.objects_end; ++i) {
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
          for (auto i = group.objects_begin; i != group.objects_end; ++i) {
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

  void color_neighbor_group(std::size_t group_index) {
    auto const &group = _neighbor_groups.group(group_index);
    auto const begin = group.neighbor_pairs_begin;
    auto const end = group.neighbor_pairs_end;
    if (begin == end) {
      return;
    }
    for (auto i = begin; i != end; ++i) {
      _neighbor_groups.neighbor_pair(i)->color = color_unmarked;
    }
    auto const seed_pair = _neighbor_groups.neighbor_pair(begin);
    seed_pair->color = color_marked;
    _coloring_fringe.emplace_back(seed_pair);
    do {
      auto const pair = _coloring_fringe.front();
      _coloring_fringe.pop_front();
      auto neighbors = std::array<std::span<Neighbor_pair *const>, 2>{};
      switch (pair->type) {
      case Object_pair_type::particle_particle:
        neighbors[0] =
            get_neighbor_pairs(get_data(Particle_handle{pair->objects[0]}));
        neighbors[1] =
            get_neighbor_pairs(get_data(Particle_handle{pair->objects[1]}));
        break;
      case Object_pair_type::particle_rigid_body:
        neighbors[0] =
            get_neighbor_pairs(get_data(Particle_handle{pair->objects[0]}));
        neighbors[1] =
            get_neighbor_pairs(get_data(Rigid_body_handle{pair->objects[1]}));
        break;
      case Object_pair_type::particle_static_body:
        neighbors[0] =
            get_neighbor_pairs(get_data(Particle_handle{pair->objects[0]}));
        break;
      case Object_pair_type::rigid_body_rigid_body:
        neighbors[0] =
            get_neighbor_pairs(get_data(Rigid_body_handle{pair->objects[0]}));
        neighbors[1] =
            get_neighbor_pairs(get_data(Rigid_body_handle{pair->objects[1]}));
        break;
      case Object_pair_type::rigid_body_static_body:
        neighbors[0] =
            get_neighbor_pairs(get_data(Rigid_body_handle{pair->objects[0]}));
        break;
      }
      _coloring_bits.reset();
      for (auto i{0}; i != 2; ++i) {
        for (auto const neighbor : neighbors[i]) {
          if (neighbor->color == color_unmarked) {
            neighbor->color = color_marked;
            _coloring_fringe.emplace_back(neighbor);
          } else if (neighbor->color != color_marked) {
            _coloring_bits.set(neighbor->color);
          }
        }
      }
      for (auto i{std::size_t{}}; i != max_colors; ++i) {
        if (!_coloring_bits.get(i)) {
          auto const color = static_cast<std::uint16_t>(i);
          pair->color = color;
          _color_groups.count(color);
          break;
        }
      }
      if (pair->color == color_marked) {
        throw std::runtime_error{"Failed to color neighbor group"};
      }
    } while (!_coloring_fringe.empty());
  }

  void assign_color_groups() {
    for (auto const i : _neighbor_group_awake_indices) {
      auto const &group = _neighbor_groups.group(i);
      auto const begin = group.neighbor_pairs_begin;
      auto const end = group.neighbor_pairs_end;
      for (auto j = begin; j != end; ++j) {
        _color_groups.push_back(_neighbor_groups.neighbor_pair(j));
      }
    }
  }

  void integrate(float delta_time,
                 float velocity_damping_factor,
                 float waking_motion_smoothing_factor) noexcept {
    for (auto const i : _neighbor_group_awake_indices) {
      integrate_neighbor_group(i,
                               delta_time,
                               velocity_damping_factor,
                               waking_motion_smoothing_factor);
    }
  }

  void integrate_neighbor_group(std::size_t group_index,
                                float delta_time,
                                float velocity_damping_factor,
                                float waking_motion_smoothing_factor) {
    auto const &group = _neighbor_groups.group(group_index);
    for (auto i = group.objects_begin; i != group.objects_end; ++i) {
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
    data->waking_motion =
        min((1.0f - waking_motion_smoothing_factor) * data->waking_motion +
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
        min((1.0f - waking_motion_smoothing_factor) * data->waking_motion +
                waking_motion_smoothing_factor *
                    (length_squared(data->velocity) +
                     length_squared(data->angular_velocity)),
            waking_motion_limit);
  }

  void solve_positions(util::Thread_pool &thread_pool,
                       Solve_state &solve_state) {
    auto solve_chunk_index = std::size_t{};
    for (auto j = std::size_t{}; j != max_colors; ++j) {
      auto const color = static_cast<std::uint16_t>(j);
      auto const group = _color_groups.group(color);
      if (!group.empty()) {
        auto const solve_chunk_count =
            (group.size() + max_solve_chunk_size - 1) / max_solve_chunk_size;
        auto latch = std::latch{static_cast<std::ptrdiff_t>(solve_chunk_count)};
        solve_state.latch = &latch;
        for (auto k = std::size_t{}; k != solve_chunk_count; ++k) {
          thread_pool.push(&_position_solve_tasks[solve_chunk_index + k]);
        }
        for (;;) {
          if (latch.try_wait()) {
            break;
          }
        }
        solve_chunk_index += solve_chunk_count;
      } else {
        break;
      }
    }
  }

  void derive_velocities(float inverse_delta_time) {
    for (auto const i : _neighbor_group_awake_indices) {
      derive_neighbor_group_velocities(i, inverse_delta_time);
    }
  }

  void derive_neighbor_group_velocities(std::size_t group_index,
                                        float inverse_delta_time) {
    auto const &group = _neighbor_groups.group(group_index);
    for (auto i = group.objects_begin; i != group.objects_end; ++i) {
      std::visit(
          [&](auto &&object) { derive_velocity(object, inverse_delta_time); },
          _neighbor_groups.object(i));
    }
  }

  void derive_velocity(Particle_handle particle, float inverse_delta_time) {
    auto const data = _particles.data(particle);
    data->velocity =
        (data->position - data->previous_position) * inverse_delta_time;
  }

  void derive_velocity(Rigid_body_handle rigid_body, float inverse_delta_time) {
    auto const data = _rigid_bodies.data(rigid_body);
    data->velocity =
        (data->position - data->previous_position) * inverse_delta_time;
    auto const delta_orientation =
        data->orientation * conjugate(data->previous_orientation);
    data->angular_velocity = 2.0f * delta_orientation.v * inverse_delta_time;
    data->angular_velocity *= delta_orientation.w >= 0.0f ? 1.0f : -1.0f;
  }

  void solve_velocities(util::Thread_pool &thread_pool,
                        Solve_state &solve_state) {
    auto solve_chunk_index = std::size_t{};
    for (auto j = std::size_t{}; j != max_colors; ++j) {
      auto const color = static_cast<std::uint16_t>(j);
      auto const group = _color_groups.group(color);
      if (!group.empty()) {
        auto const solve_chunk_count =
            (group.size() + max_solve_chunk_size - 1) / max_solve_chunk_size;
        auto latch = std::latch{static_cast<std::ptrdiff_t>(solve_chunk_count)};
        solve_state.latch = &latch;
        for (auto k = std::size_t{}; k != solve_chunk_count; ++k) {
          thread_pool.push(&_velocity_solve_tasks[solve_chunk_index + k]);
        }
        for (;;) {
          if (latch.try_wait()) {
            break;
          }
        }
        solve_chunk_index += solve_chunk_count;
      } else {
        break;
      }
    }
  }

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

  bool is_marked(Particle_handle particle) const noexcept {
    return _particles.data(particle)->marked;
  }

  bool is_marked(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body)->marked;
  }

  void set_marked(Particle_handle particle, bool marked = true) noexcept {
    _particles.data(particle)->marked = marked;
  }

  void set_marked(Rigid_body_handle rigid_body, bool marked = true) noexcept {
    _rigid_bodies.data(rigid_body)->marked = marked;
  }

  void set_unmarked(Particle_handle particle) noexcept {
    set_marked(particle, false);
  }

  void set_unmarked(Rigid_body_handle rigid_body) noexcept {
    set_marked(rigid_body, false);
  }

  std::span<Neighbor_pair *const>
  get_neighbor_pairs(Particle_data *particle) const noexcept {
    return {particle->neighbor_pairs, particle->neighbor_count};
  }

  std::span<Neighbor_pair *const>
  get_neighbor_pairs(Rigid_body_data *rigid_body) const noexcept {
    return {rigid_body->neighbor_pairs, rigid_body->neighbor_count};
  }

  Particle_data const *get_data(Particle_handle particle) const noexcept {
    return _particles.data(particle);
  }

  Particle_data *get_data(Particle_handle particle) noexcept {
    return _particles.data(particle);
  }

  Rigid_body_data const *get_data(Rigid_body_handle rigid_body) const noexcept {
    return _rigid_bodies.data(rigid_body);
  }

  Rigid_body_data *get_data(Rigid_body_handle rigid_body) noexcept {
    return _rigid_bodies.data(rigid_body);
  }

  Static_body_data const *
  get_data(Static_body_handle static_body) const noexcept {
    return _static_bodies.data(static_body);
  }

  Static_body_data *get_data(Static_body_handle static_body) noexcept {
    return _static_bodies.data(static_body);
  }

  Block _block;
  Particle_storage _particles;
  Static_body_storage _static_bodies;
  Rigid_body_storage _rigid_bodies;
  Aabb_tree<Aabb_tree_payload_t> _aabb_tree;
  List<Neighbor_pair> _neighbor_pairs;
  List<Neighbor_pair *> _neighbor_pair_ptrs;
  Neighbor_group_storage _neighbor_groups;
  List<std::uint32_t> _neighbor_group_awake_indices;
  Bit_list _coloring_bits;
  Queue<Neighbor_pair *> _coloring_fringe;
  Color_group_storage _color_groups;
  List<Contact> _contacts;
  List<Solve_chunk> _solve_chunks;
  List<Position_solve_task> _position_solve_tasks;
  List<Velocity_solve_task> _velocity_solve_tasks;
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
  return _impl->create_static_body(create_info);
}

void World::destroy_rigid_body(Static_body_handle static_rigid_body) {
  _impl->destroy_static_body(static_rigid_body);
}

void World::simulate(World_simulate_info const &simulate_info) {
  return _impl->simulate(*this, simulate_info);
}
} // namespace physics
} // namespace marlon