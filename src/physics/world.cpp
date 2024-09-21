#include "world.h"

#include <chrono>
#include <latch>

#include "../math/scalar.h"
#include "../util/list.h"
#include "../util/map.h"
#include "broadphase.h"
#include "constraint.h"
#include "contact.h"
#include "narrowphase.h"
#include "util/thread_pool.h"

namespace marlon {
namespace physics {
using namespace math;
using namespace util;
namespace {
// auto constexpr color_unmarked{static_cast<std::uint16_t>(-1)};
// auto constexpr color_marked{static_cast<std::uint16_t>(-2)};
// auto constexpr reserved_colors{std::size_t{2}};
// auto constexpr max_colors = (std::size_t{1} << 16) - reserved_colors;

class Neighbor_group_storage {
  using Allocator = Stack_allocator<>;

public:
  struct Group {
    Size objects_begin;
    Size objects_end;
  };

  template <typename Allocator>
  static std::pair<Block, Neighbor_group_storage>
  make(Allocator &allocator, Size max_object_count, Size max_group_count) {
    auto const block = allocator.alloc(memory_requirement(max_object_count, max_group_count));
    return {block, Neighbor_group_storage{block, max_object_count, max_group_count}};
  }

  static constexpr Size memory_requirement(Size max_object_count, Size max_group_count) {
    return Allocator::memory_requirement({
        decltype(_objects)::memory_requirement(max_object_count),
        decltype(_groups)::memory_requirement(max_group_count),
    });
  }

  constexpr Neighbor_group_storage() noexcept = default;

  Neighbor_group_storage(Block block, Size max_object_count, Size max_group_count)
      : Neighbor_group_storage{block.begin, max_object_count, max_group_count} {}

  Neighbor_group_storage(std::byte *block_begin, Size max_object_count, Size max_group_count) {
    auto allocator = Allocator{{block_begin, memory_requirement(max_object_count, max_group_count)}};
    _objects = decltype(_objects)::make(allocator, max_object_count).second;
    _groups = decltype(_groups)::make(allocator, max_group_count).second;
  }

  Size object_count() const noexcept {
    return _objects.size();
  }

  std::variant<Particle, Rigid_body> object_specific(Size index) const noexcept {
    return std::visit(
        [&](auto const object) -> std::variant<Particle, Rigid_body> {
          using T = std::decay_t<decltype(object)>;
          if constexpr (std::is_same_v<T, Particle> || std::is_same_v<T, Rigid_body>) {
            return object;
          } else {
            unreachable();
            throw;
          }
        },
        object_generic(index).specific());
  }

  Object object_generic(Size index) const noexcept {
    return _objects[index];
  }

  Size group_count() const noexcept {
    return _groups.size();
  }

  Group const &group(util::Size group_index) const noexcept {
    return _groups[group_index];
  }

  void clear() noexcept {
    _objects.clear();
    _groups.clear();
  }

  void begin_group() {
    auto const objects_index = _objects.size();
    _groups.push_back({
        .objects_begin = objects_index,
        .objects_end = objects_index,
    });
  }

  void add_to_group(Particle object) {
    _objects.push_back(object.generic());
    ++_groups.back().objects_end;
  }

  void add_to_group(Rigid_body object) {
    _objects.push_back(object.generic());
    ++_groups.back().objects_end;
  }

private:
  List<Object> _objects;
  List<Group> _groups;
};

// class Color_group_storage {
//   using Allocator = Stack_allocator<>;

// public:
//   template <typename Allocator>
//   static std::pair<Block, Color_group_storage>
//   make(Allocator &allocator, std::size_t max_neighbor_pairs) {
//     auto const block =
//     allocator.alloc(memory_requirement(max_neighbor_pairs)); return {block,
//     Color_group_storage{block, max_neighbor_pairs}};
//   }

//   static constexpr std::size_t
//   memory_requirement(std::size_t max_neighbor_pairs) noexcept {
//     return Allocator::memory_requirement({
//         decltype(_neighbor_pairs)::memory_requirement(max_neighbor_pairs),
//         decltype(_groups)::memory_requirement(max_colors),
//     });
//   }

//   constexpr Color_group_storage() = default;

//   explicit Color_group_storage(Block block, std::size_t max_neighbor_pairs)
//       : Color_group_storage{block.begin, max_neighbor_pairs} {}

//   explicit Color_group_storage(void *block, std::size_t max_neighbor_pairs) {
//     auto allocator =
//         Allocator{make_block(block, memory_requirement(max_neighbor_pairs))};
//     _neighbor_pairs =
//         List<Object_pair *>::make(allocator, max_neighbor_pairs).second;
//     _groups = List<Group>::make(allocator, max_colors).second;
//     _groups.resize(max_colors);
//   }

//   std::span<Object_pair *const> group(std::uint16_t color) const noexcept {
//     return {_neighbor_pairs.data() + _groups[color].neighbor_pairs_begin,
//             _neighbor_pairs.data() + _groups[color].neighbor_pairs_end};
//   }

//   void clear() noexcept {
//     _neighbor_pairs.clear();
//     _groups.clear();
//     _groups.resize(max_colors);
//   }

//   void count(std::uint16_t color) noexcept {
//     ++_groups[color].neighbor_pairs_end;
//   }

//   void reserve() {
//     for (auto &group : _groups) {
//       if (group.neighbor_pairs_end == 0) {
//         return;
//       }
//       auto const index = static_cast<std::uint32_t>(_neighbor_pairs.size());
//       _neighbor_pairs.resize(_neighbor_pairs.size() +
//       group.neighbor_pairs_end); group.neighbor_pairs_begin = index;
//       group.neighbor_pairs_end = index;
//     }
//   }

//   void push_back(Object_pair *neighbor_pair) {
//     _neighbor_pairs[_groups[neighbor_pair->color()].neighbor_pairs_end++] =
//         neighbor_pair;
//   }

// private:
//   struct Group {
//     std::uint32_t neighbor_pairs_begin{};
//     std::uint32_t neighbor_pairs_end{};
//   };

//   List<Object_pair *> _neighbor_pairs;
//   List<Group> _groups;
// };

class Narrowphase_task : public util::Task {
  struct Object_derived_data {
    Vec3f position;
    Quatf orientation;
  };

public:
  struct Intrinsic_state {
    Object_storage *objects;
    std::latch *latch;
  };

  explicit Narrowphase_task(Intrinsic_state const *intrinsic_state,
                            std::span<std::pair<Object_pair, Contact_manifold> *const> items) noexcept
      : _intrinsic_state{intrinsic_state}, _items{items} {}

  void run(Size) final {
    for (auto const &p : _items) {
      auto const objects = p->first;
      auto &contact_manifold = p->second;
      // auto const objects = Object_handle_pair{it->first};
      if (auto const narrowphase_result = do_narrowphase(objects)) {
        auto object_derived_data = std::array<Object_derived_data, 2>{};
        std::visit(
            [&](auto &&first_object) {
              object_derived_data[0] = derived_data(_intrinsic_state->objects->data(first_object));
            },
            objects.first_specific());
        std::visit(
            [&](auto &&second_object) {
              object_derived_data[1] = derived_data(_intrinsic_state->objects->data(second_object));
            },
            objects.second_specific());
        auto const object_positions = std::array<Vec3f, 2>{
            object_derived_data[0].position,
            object_derived_data[1].position,
        };
        auto const object_orientations = std::array<Quatf, 2>{
            object_derived_data[0].orientation,
            object_derived_data[1].orientation,
        };
        contact_manifold.update(object_positions, object_orientations);
        contact_manifold.insert({
            .contact = Contact{narrowphase_result->normal,
                               narrowphase_result->local_positions,
                               narrowphase_result->separation},
            .initial_object_orientations = object_orientations,
        });
      } else {
        contact_manifold.clear();
      }
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  std::optional<Narrowphase_result> do_narrowphase(Object_pair generic) const noexcept {
    auto result = std::optional<Narrowphase_result>{};
    std::visit(
        [&](auto const specific) {
          result = object_object_contact(*_intrinsic_state->objects->data(specific.first),
                                         *_intrinsic_state->objects->data(specific.second));
        },
        generic.specific());
    return result;
  }

  Object_derived_data derived_data(Particle_data const *data) const noexcept {
    return {data->position(), Quatf::identity()};
  }

  Object_derived_data derived_data(Rigid_body_data const *data) const noexcept {
    return {data->position(), data->orientation()};
  }

  Object_derived_data derived_data(Static_body_data const *data) {
    return {data->position(), data->orientation()};
  }

  Intrinsic_state const *_intrinsic_state;
  std::span<std::pair<Object_pair, Contact_manifold> *const> _items;
};

class Position_solve_task : public util::Task {
public:
  struct Intrinsic_state {
    Object_storage *objects;
    std::latch *latch;
  };

  struct Work_item {
    Constraint *constraint;
    Object_pair objects;
  };

  explicit Position_solve_task(Intrinsic_state const *intrinsic_state, std::span<Work_item const> work_items)
      : _intrinsic_state{intrinsic_state}, _work_items{work_items} {}

  void run(Size /*thread_index*/) final {
    for (auto &work_item : _work_items) {
      work_item.constraint->solve_position(work_item.objects, *_intrinsic_state->objects);
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  Intrinsic_state const *_intrinsic_state;
  std::span<Work_item const> _work_items;
};

class Velocity_solve_task : public util::Task {
  struct Object_derived_data {
    Vec3f velocity;
    Vec3f angular_velocity;
    Mat3x3f rotation;
    Mat3x3f inverse_rotation;
    float inverse_mass;
    Mat3x3f inverse_inertia_tensor;
    Material material;
  };

public:
  struct Intrinsic_state {
    Object_storage *objects;
    std::latch *latch;
    float restitution_epsilon;
  };

  struct Work_item {
    Constraint *constraint;
    Object_pair objects;
  };

  explicit Velocity_solve_task(Intrinsic_state const *intrinsic_state, std::span<Work_item const> work_items)
      : _intrinsic_state{intrinsic_state}, _work_items{work_items} {}

  void run(Size /*thread_index*/) final {
    for (auto const &work_item : _work_items) {
      work_item.constraint->solve_velocity(
          work_item.objects, *_intrinsic_state->objects, _intrinsic_state->restitution_epsilon);
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  Intrinsic_state const *_intrinsic_state;
  std::span<Work_item const> _work_items;
};

// integration constants
auto constexpr velocity_damping_factor = 0.99f;
auto constexpr motion_epsilon = 0.02f;
auto constexpr motion_initializer = 2.0f * motion_epsilon;
auto constexpr motion_limit = 10.0f * motion_epsilon;
auto constexpr motion_smoothing_factor = 0.9f;
auto constexpr max_narrowphase_task_size = Size{32};
} // namespace

class World::Impl {
public:
  friend class World;

  static constexpr std::size_t memory_requirement(World_create_info const &create_info) {
    return Stack_allocator<>::memory_requirement({
        decltype(_objects)::memory_requirement(
            create_info.max_particles, create_info.max_rigid_bodies, create_info.max_static_bodies),
        decltype(_bvh)::memory_requirement(create_info.max_aabb_tree_leaf_nodes,
                                           create_info.max_aabb_tree_internal_nodes),
        decltype(_neighbor_pairs)::memory_requirement(create_info.max_neighbor_pairs),
        decltype(_neighbors)::memory_requirement(2 * create_info.max_neighbor_pairs),
        decltype(_neighbor_groups)::memory_requirement(create_info.max_particles + create_info.max_rigid_bodies,
                                                       create_info.max_neighbor_groups),
        decltype(_awake_neighbor_group_indices)::memory_requirement(create_info.max_neighbor_groups),
        // decltype(_coloring_bits)::memory_requirement(max_colors),
        // decltype(_coloring_fringe)::memory_requirement(
        //     create_info.max_neighbor_pairs),
        // decltype(_color_groups)::memory_requirement(
        //     create_info.max_neighbor_pairs),
        decltype(_contact_manifolds)::memory_requirement(create_info.max_neighbor_pairs),
        decltype(_awake_contact_manifolds)::memory_requirement(create_info.max_neighbor_pairs),
        decltype(_narrowphase_tasks)::memory_requirement(
            (create_info.max_neighbor_pairs + max_narrowphase_task_size - 1) / max_narrowphase_task_size),
    });
  }

  explicit Impl(World_create_info const &create_info)
      : _threads{create_info.worker_thread_count},
        _block{util::System_allocator{}.alloc(memory_requirement(create_info))},
        _gravitational_acceleration{create_info.gravitational_acceleration} {
    util::assign_merged(
        Stack_allocator<>{_block},
        std::tie(_objects,
                 _bvh,
                 _neighbor_pairs,
                 _neighbors,
                 _neighbor_groups,
                 _awake_neighbor_group_indices,
                 _contact_manifolds,
                 _awake_contact_manifolds,
                 _narrowphase_tasks),
        // _objects
        std::tuple{create_info.max_particles, create_info.max_rigid_bodies, create_info.max_static_bodies},
        // _bvh
        std::tuple{create_info.max_aabb_tree_leaf_nodes, create_info.max_aabb_tree_internal_nodes},
        // _neighbor_pairs
        std::tuple{create_info.max_neighbor_pairs},
        // _neighbors
        std::tuple{2 * create_info.max_neighbor_pairs},
        // _neighbor_groups
        std::tuple{create_info.max_particles + create_info.max_rigid_bodies, create_info.max_neighbor_groups},
        // _awake_neighbor_group_indices
        std::tuple{create_info.max_neighbor_groups},
        // _contact_manifolds
        std::tuple{create_info.max_neighbor_pairs},
        // _awake_contact_manifolds
        std::tuple{create_info.max_neighbor_pairs},
        // _narrowphase_tasks
        std::tuple{(create_info.max_neighbor_pairs + max_narrowphase_task_size - 1) / max_narrowphase_task_size});
    _narrowphase_task_intrinsic_state.objects = &_objects;
  }

  ~Impl() {
    _narrowphase_tasks = {};
    _awake_contact_manifolds = {};
    _contact_manifolds = {};
    _awake_neighbor_group_indices = {};
    _neighbor_groups = {};
    _neighbors = {};
    _neighbor_pairs = {};
    _bvh = {};
    _objects = {};
    util::System_allocator{}.free(_block);
  }

  template <typename T> auto data(T object) const {
    return _objects.data(object);
  }

  template <typename T> auto data(T object) {
    return _objects.data(object);
  }

  Particle create_particle(Particle_create_info const &create_info) {
    auto const bvh_node = _bvh.create_leaf({}, {});
    try {
      auto const particle = _objects.create_particle(bvh_node,
                                                     create_info.motion_callback,
                                                     create_info.position,
                                                     create_info.velocity,
                                                     motion_initializer,
                                                     1.0f / create_info.mass,
                                                     create_info.radius,
                                                     create_info.material);
      bvh_node->payload = particle.generic();
      return particle;
    } catch (...) {
      _bvh.destroy_leaf(bvh_node);
      throw;
    }
  }

  void destroy_particle(Particle particle) {
    _bvh.destroy_leaf(data(particle)->bvh_node());
    _objects.destroy(particle);
  }

  Rigid_body create_rigid_body(Rigid_body_create_info const &create_info) {
    auto const bvh_node = _bvh.create_leaf({}, {});
    try {
      auto const rigid_body = _objects.create_rigid_body(bvh_node,
                                                         create_info.motion_callback,
                                                         create_info.position,
                                                         create_info.velocity,
                                                         create_info.orientation,
                                                         create_info.angular_velocity,
                                                         motion_initializer,
                                                         1.0f / create_info.mass,
                                                         inverse(create_info.inertia_tensor),
                                                         create_info.shape,
                                                         create_info.material);
      bvh_node->payload = rigid_body.generic();
      return rigid_body;
    } catch (...) {
      _bvh.destroy_leaf(bvh_node);
      throw;
    }
  }

  void destroy_rigid_body(Rigid_body rigid_body) {
    _bvh.destroy_leaf(data(rigid_body)->bvh_node());
    _objects.destroy(rigid_body);
  }

  Static_body create_static_body(Static_body_create_info const &create_info) {
    auto const bvh_node =
        _bvh.create_leaf(bounds(create_info.shape, Mat3x4f::rigid(create_info.position, create_info.orientation)), {});
    try {
      auto const static_body = _objects.create_static_body(
          bvh_node, create_info.position, create_info.orientation, create_info.shape, create_info.material);
      bvh_node->payload = static_body.generic();
      return static_body;
    } catch (...) {
      _bvh.destroy_leaf(bvh_node);
      throw;
    }
  }

  void destroy_static_body(Static_body static_body) {
    _bvh.destroy_leaf(data(static_body)->bvh_node());
    _objects.destroy(static_body);
  }

  World_simulate_result simulate(World const &world, World_simulate_info const &simulate_info) {
    using clock = std::chrono::system_clock;
    using duration = std::chrono::duration<double>;
    auto result = World_simulate_result{};
    auto const broadphase_begin = clock::now();
    build_aabb_tree(simulate_info.delta_time);
    // clear_neighbors();
    find_neighbors();
    // assign_neighbors();
    find_neighbor_groups();
    find_awake_neighbor_groups();
    find_awake_contact_manifolds();
    make_narrowphase_tasks();
    auto const broadphase_end = clock::now();
    result.broadphase_wall_time = std::chrono::duration_cast<duration>(broadphase_end - broadphase_begin).count();
    // _color_groups.reserve();
    // assign_color_groups();
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    // for (auto i = std::size_t{}; i != max_colors; ++i) {
    //   auto const color = static_cast<std::uint16_t>(i);
    //   auto const group = _color_groups.group(color);
    //   if (!group.empty()) {
    //     for (auto j = std::size_t{}; j < group.size();
    //          j += max_solve_chunk_size) {
    //       auto const chunk_size = min(group.size() - j,
    //       max_solve_chunk_size); _solve_chunks.push_back(
    //           {.pairs = group.data() + j,
    //            .contacts = _contacts.data() + _contacts.size(),
    //            .size = chunk_size});
    //       _contacts.resize(_contacts.size() + chunk_size);
    //       _position_solve_tasks.emplace_back(&solve_state,
    //                                          &_solve_chunks.back());
    //       _velocity_solve_tasks.emplace_back(&solve_state,
    //                                          &_solve_chunks.back());
    //     }
    //   } else {
    //     break;
    //   }
    // }
    auto const time_compensated_velocity_damping_factor = pow(velocity_damping_factor, h);
    auto const time_compensating_waking_motion_smoothing_factor = 1.0f - pow(1.0f - motion_smoothing_factor, h);
    auto const restitution_separating_velocity_epsilon = 2.0f * h * length(_gravitational_acceleration);
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      auto const integration_begin = clock::now();
      integrate(h, time_compensated_velocity_damping_factor, time_compensating_waking_motion_smoothing_factor);
      auto const integration_end = clock::now();
      run_narrowphase_tasks();
      auto const narrowphase_end = clock::now();
      solve_positions();
      auto const position_solve_end = clock::now();
      solve_velocities(restitution_separating_velocity_epsilon);
      auto const velocity_solve_end = clock::now();
      result.integration_wall_time += std::chrono::duration_cast<duration>(integration_end - integration_begin).count();
      result.narrowphase_wall_time += std::chrono::duration_cast<duration>(narrowphase_end - integration_end).count();
      result.position_solve_wall_time +=
          std::chrono::duration_cast<duration>(position_solve_end - narrowphase_end).count();
      result.velocity_solve_wall_time +=
          std::chrono::duration_cast<duration>(velocity_solve_end - position_solve_end).count();
    }
    auto const simulate_end = clock::now();
    result.total_wall_time = std::chrono::duration_cast<duration>(simulate_end - broadphase_begin).count();
    call_motion_callbacks(world);
    return result;
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const constant_safety_term = 0.0f;
    auto const velocity_safety_factor = 2.0f;
    auto const gravity_safety_factor = 2.0f;
    auto const gravity_safety_term =
        gravity_safety_factor * length(_gravitational_acceleration) * delta_time * delta_time;
    _objects.for_each_particle([&](Particle object) {
      auto const object_data = data(object);
      auto const half_extent = object_data->radius() + constant_safety_term +
                               velocity_safety_factor * length(object_data->velocity()) * delta_time +
                               gravity_safety_term;
      object_data->bvh_node()->bounds = expand(Aabb3f{object_data->position(), object_data->position()}, half_extent);
    });
    _objects.for_each_rigid_body([&](Rigid_body object) {
      auto const object_data = data(object);
      auto const transform = Mat3x4f::rigid(object_data->position(), object_data->orientation());
      object_data->bvh_node()->bounds =
          expand(bounds(object_data->shape(), transform),
                 constant_safety_term + velocity_safety_factor * length(object_data->velocity()) * delta_time +
                     gravity_safety_term);
    });
    _bvh.build();
  }

  void find_neighbors() {
    auto const reset_neighbors = [&](auto const object) { data(object)->reset_neighbors(); };
    _objects.for_each_particle(reset_neighbors);
    _objects.for_each_rigid_body(reset_neighbors);
    _neighbor_pairs.clear();
    _neighbors.clear();
    _neighbor_groups.clear();
    _bvh.for_each_overlapping_leaf_pair([this](Object first_generic, Object second_generic) {
      visit(
          [&](auto const first_specific, auto const second_specific) {
            using T = std::decay_t<decltype(first_specific)>;
            using U = std::decay_t<decltype(second_specific)>;
            if constexpr (!std::is_same_v<T, Static_body> || !std::is_same_v<U, Static_body>) {
              auto const pair = _neighbor_pairs.emplace_back(first_specific, second_specific);
              auto const it =
                  _contact_manifolds.emplace(std::piecewise_construct, std::tuple{pair}, std::tuple{}).first;
              it->second.marked(true);
              if constexpr (!std::is_same_v<T, Static_body>) {
                data(first_specific)->count_neighbor();
              }
              if constexpr (!std::is_same_v<U, Static_body>) {
                data(second_specific)->count_neighbor();
              }
            }
          },
          first_generic.specific(),
          second_generic.specific());
    });
    for (auto it = _contact_manifolds.begin(); it != _contact_manifolds.end();) {
      if (it->second.marked()) {
        it->second.marked(false);
        ++it;
      } else {
        it = _contact_manifolds.erase(it);
      }
    }
    auto const reserve_neighbors = [this](auto const object) { data(object)->reserve_neighbors(_neighbors); };
    _objects.for_each_particle(reserve_neighbors);
    _objects.for_each_rigid_body(reserve_neighbors);
    for (auto &pair : _neighbor_pairs) {
      std::visit([&](auto const object) { data(object)->push_neighbor(pair.second_generic()); }, pair.first_specific());
      std::visit(
          [&](auto const object) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(object)>, Static_body>) {
              data(object)->push_neighbor(pair.first_generic());
            }
          },
          pair.second_specific());
    }
  }

  void find_neighbor_groups() {
    auto const unmark = [&](auto object) { data(object)->marked(false); };
    _objects.for_each_particle(unmark);
    _objects.for_each_rigid_body(unmark);
    auto const visitor = [this](auto &&object) {
      auto const object_data = data(object);
      for (auto const generic_neighbor : object_data->neighbors()) {
        std::visit(
            [&](auto &&specific_neighbor) {
              using U = std::decay_t<decltype(specific_neighbor)>;
              if constexpr (!std::is_same_v<U, Static_body>) {
                auto const neighbor_data = data(specific_neighbor);
                if (!neighbor_data->marked()) {
                  neighbor_data->marked(true);
                  _neighbor_groups.add_to_group(specific_neighbor);
                }
              }
            },
            generic_neighbor.specific());
      }
    };
    auto fringe_index = util::Size{};
    auto const find_neighbor_group = [&, this](auto const seed_object) {
      auto const seed_data = data(seed_object);
      if (!seed_data->marked()) {
        seed_data->marked(true);
        _neighbor_groups.begin_group();
        _neighbor_groups.add_to_group(seed_object);
        do {
          visit(visitor, _neighbor_groups.object_specific(fringe_index));
        } while (++fringe_index != _neighbor_groups.object_count());
      }
    };
    _objects.for_each_particle(find_neighbor_group);
    _objects.for_each_rigid_body(find_neighbor_group);
  }

  void find_awake_neighbor_groups() {
    _awake_neighbor_group_indices.clear();
    auto const group_count = _neighbor_groups.group_count();
    for (auto group_index = util::Size{}; group_index < group_count; ++group_index) {
      auto const &group = _neighbor_groups.group(group_index);
      auto contains_awake = false;
      auto contains_asleep = false;
      auto sleepable = true;
      for (auto i = group.objects_begin; (sleepable || !contains_awake || !contains_asleep) && i != group.objects_end;
           ++i) {
        std::visit(
            [&](auto &&object) {
              auto const object_data = data(object);
              if (object_data->asleep()) {
                contains_asleep = true;
              } else {
                contains_awake = true;
                if (object_data->motion() > motion_epsilon) {
                  sleepable = false;
                }
              }
            },
            _neighbor_groups.object_specific(i));
      }
      if (contains_awake) {
        if (sleepable) {
          for (auto i = group.objects_begin; i != group.objects_end; ++i) {
            std::visit(
                [&](auto &&object) {
                  auto const object_data = data(object);
                  if (object_data->awake()) {
                    object_data->sleep();
                  }
                },
                _neighbor_groups.object_specific(i));
          }
        } else {
          if (contains_asleep) {
            for (auto i = group.objects_begin; i != group.objects_end; ++i) {
              std::visit(
                  [&](auto &&object) {
                    auto const object_data = data(object);
                    if (object_data->asleep()) {
                      object_data->wake(motion_initializer);
                    }
                  },
                  _neighbor_groups.object_specific(i));
            }
          }
          _awake_neighbor_group_indices.emplace_back(group_index);
        }
      }
    }
  }

  void find_awake_contact_manifolds() {
    _awake_contact_manifolds.clear();
    for (auto const group_index : _awake_neighbor_group_indices) {
      auto const group = _neighbor_groups.group(group_index);
      for (auto object_index = group.objects_begin; object_index != group.objects_end; ++object_index) {
        std::visit([&](auto const object) { data(object)->marked(false); },
                   _neighbor_groups.object_specific(object_index));
      }
      for (auto object_index = group.objects_begin; object_index != group.objects_end; ++object_index) {
        std::visit(
            [&](auto const object) {
              auto const object_data = data(object);
              object_data->marked(true);
              for (auto const neighbor_generic : object_data->neighbors()) {
                std::visit(
                    [&](auto const neighbor_specific) {
                      using U = std::decay_t<decltype(neighbor_specific)>;
                      if constexpr (std::is_same_v<U, Static_body>) {
                        _awake_contact_manifolds.emplace_back(
                            &*_contact_manifolds.find(Object_pair{object, neighbor_specific}));
                      } else if (!data(neighbor_specific)->marked()) {
                        _awake_contact_manifolds.emplace_back(
                            &*_contact_manifolds.find(Object_pair{object, neighbor_specific}));
                      }
                    },
                    neighbor_generic.specific());
              }
            },
            _neighbor_groups.object_specific(object_index));
      }
    }
    // std::ranges::sort(_awake_contact_manifolds);
  }

  void make_narrowphase_tasks() noexcept {
    auto const complete_task_count = _awake_contact_manifolds.size() / max_narrowphase_task_size;
    auto const partial_task_size = _awake_contact_manifolds.size() % max_narrowphase_task_size;
    _narrowphase_tasks.clear();
    for (auto i = Size{}; i != complete_task_count; ++i) {
      _narrowphase_tasks.emplace_back(&_narrowphase_task_intrinsic_state,
                                      std::span{&_awake_contact_manifolds[i * max_narrowphase_task_size],
                                                static_cast<std::size_t>(max_narrowphase_task_size)});
    }
    if (partial_task_size != 0) {
      _narrowphase_tasks.emplace_back(
          &_narrowphase_task_intrinsic_state,
          std::span{&_awake_contact_manifolds[complete_task_count * max_narrowphase_task_size],
                    static_cast<std::size_t>(partial_task_size)});
    }
  }

  // void color_neighbor_group(std::size_t group_index) {
  //   auto const &group = _neighbor_groups.group(group_index);
  //   auto const begin = group.neighbor_pairs_begin;
  //   auto const end = group.neighbor_pairs_end;
  //   if (begin == end) {
  //     return;
  //   }
  //   for (auto i = begin; i != end; ++i) {
  //     _neighbor_groups.neighbor_pair(i)->color(color_unmarked);
  //   }
  //   auto const seed_pair = _neighbor_groups.neighbor_pair(begin);
  //   seed_pair->color(color_marked);
  //   _coloring_fringe.emplace_back(seed_pair);
  //   do {
  //     auto const pair = _coloring_fringe.front();
  //     _coloring_fringe.pop_front();
  //     auto neighbors = std::array<std::span<Object_pair *const>, 2>{};
  //     visit(
  //         [&](auto &&object) {
  //           neighbors[0] = get_neighbor_pairs(get_data(object));
  //         },
  //         pair->first());
  //     visit(
  //         [&](auto &&object) {
  //           neighbors[1] = get_neighbor_pairs(get_data(object));
  //         },
  //         pair->second());
  //     _coloring_bits.reset();
  //     for (auto i{0}; i != 2; ++i) {
  //       for (auto const neighbor : neighbors[i]) {
  //         if (neighbor->color() == color_unmarked) {
  //           neighbor->color(color_marked);
  //           _coloring_fringe.emplace_back(neighbor);
  //         } else if (neighbor->color() != color_marked) {
  //           _coloring_bits.set(neighbor->color());
  //         }
  //       }
  //     }
  //     for (auto i{std::size_t{}}; i != max_colors; ++i) {
  //       if (!_coloring_bits.get(i)) {
  //         auto const color = static_cast<std::uint16_t>(i);
  //         pair->color(color);
  //         _color_groups.count(color);
  //         break;
  //       }
  //     }
  //     if (pair->color() == color_marked) {
  //       throw std::runtime_error{"Failed to color neighbor group"};
  //     }
  //   } while (!_coloring_fringe.empty());
  // }

  // void assign_color_groups() {
  //   for (auto const i : _awake_neighbor_group_indices) {
  //     auto const &group = _neighbor_groups.group(i);
  //     auto const begin = group.neighbor_pairs_begin;
  //     auto const end = group.neighbor_pairs_end;
  //     for (auto j = begin; j != end; ++j) {
  //       _color_groups.push_back(_neighbor_groups.neighbor_pair(j));
  //     }
  //   }
  // }

  void integrate(float delta_time, float velocity_damping_factor, float waking_motion_smoothing_factor) noexcept {
    for (auto const i : _awake_neighbor_group_indices) {
      integrate_neighbor_group(i, delta_time, velocity_damping_factor, waking_motion_smoothing_factor);
    }
  }

  void integrate_neighbor_group(util::Size group_index,
                                float delta_time,
                                float damping_factor,
                                float motion_smoothing_factor) {
    auto const &group = _neighbor_groups.group(group_index);
    for (auto i = group.objects_begin; i != group.objects_end; ++i) {
      std::visit(
          [&](auto &&object) {
            data(object)->integrate({
                .delta_velocity = delta_time * _gravitational_acceleration,
                .delta_time = delta_time,
                .damping_factor = damping_factor,
                .motion_smoothing_factor = motion_smoothing_factor,
                .motion_limit = motion_limit,
            });
          },
          _neighbor_groups.object_specific(i));
    }
  }

  void run_narrowphase_tasks() {
    if (_threads.empty()) {
      _narrowphase_task_intrinsic_state.latch = nullptr;
      for (auto &task : _narrowphase_tasks) {
        task.run({});
      }
    } else {
      auto latch = std::latch{_narrowphase_tasks.size()};
      _narrowphase_task_intrinsic_state.latch = &latch;
      for (auto &task : _narrowphase_tasks) {
        _threads.push_silent(&task);
      }
      // _threads.set_scheduling_policy(Scheduling_policy::spin);
      _threads.notify();
      for (;;) {
        if (latch.try_wait()) {
          return;
        }
      }
      // _threads.set_scheduling_policy(Scheduling_policy::block);
    }
  }

  void solve_positions() {
    auto const intrinsic_state = Position_solve_task::Intrinsic_state{
        .objects = &_objects,
        .latch = nullptr,
    };
    for (auto i = 0; i != 4; ++i) {
      for (auto const p : _awake_contact_manifolds) {
        auto &[objects, contact_manifold] = *p;
        for (auto &cached_contact : contact_manifold.contacts()) {
          auto const work_item = Position_solve_task::Work_item{
              .constraint = &cached_contact.contact,
              .objects = objects,
          };
          Position_solve_task{&intrinsic_state, std::span{&work_item, 1u}}.run({});
        }
      }
    }
  }

  void solve_velocities(float restitution_epsilon) {
    auto const intrinsic_state = Velocity_solve_task::Intrinsic_state{
        .objects = &_objects,
        .latch = nullptr,
        .restitution_epsilon = restitution_epsilon,
    };
    for (auto i = 0; i != 1; ++i) {
      for (auto const p : _awake_contact_manifolds) {
        auto &[objects, contact_manifold] = *p;
        for (auto &cached_contact : contact_manifold.contacts()) {
          auto const work_item = Velocity_solve_task::Work_item{
              .constraint = &cached_contact.contact,
              .objects = objects,
          };
          Velocity_solve_task{&intrinsic_state, {&work_item, 1u}}.run({});
        }
      }
    }
  }

  void call_motion_callbacks(World const &world) {
    auto call_object_motion_callback = [&](auto object) {
      using T = decltype(object);
      auto const object_data = data(object);
      if (object_data->awake() && object_data->motion_callback() != nullptr) {
        if constexpr (std::is_same_v<T, Particle>) {
          object_data->motion_callback()->on_particle_motion(world, object);
        } else {
          static_assert(std::is_same_v<T, Rigid_body>);
          object_data->motion_callback()->on_rigid_body_motion(world, object);
        }
      }
    };
    _objects.for_each_particle(call_object_motion_callback);
    _objects.for_each_rigid_body(call_object_motion_callback);
  }

  Thread_pool _threads;
  Block _block;
  Object_storage _objects;
  Broadphase_bvh _bvh;
  List<Object_pair> _neighbor_pairs;
  List<Object> _neighbors;
  Neighbor_group_storage _neighbor_groups;
  List<Size> _awake_neighbor_group_indices;
  // Bit_list _coloring_bits;
  // Queue<Object_pair *> _coloring_fringe;
  // Color_group_storage _color_groups;
  // List<Contact> _contacts;
  Map<Object_pair, Contact_manifold> _contact_manifolds;
  List<std::pair<Object_pair, Contact_manifold> *> _awake_contact_manifolds;
  List<Narrowphase_task> _narrowphase_tasks;
  Narrowphase_task::Intrinsic_state _narrowphase_task_intrinsic_state;
  Vec3f _gravitational_acceleration;
};

World::World() {}

World::World(World_create_info const &create_info)
    : _impl{std::make_unique<Impl>(create_info)} {}

World::~World() {}

Particle World::create_particle(Particle_create_info const &create_info) {
  return _impl->create_particle(create_info);
}

Rigid_body World::create_rigid_body(Rigid_body_create_info const &create_info) {
  return _impl->create_rigid_body(create_info);
}

Static_body World::create_static_body(Static_body_create_info const &create_info) {
  return _impl->create_static_body(create_info);
}

void World::destroy_object(Particle particle) {
  _impl->destroy_particle(particle);
}

void World::destroy_object(Rigid_body handle) {
  _impl->destroy_rigid_body(handle);
}

void World::destroy_object(Static_body static_rigid_body) {
  _impl->destroy_static_body(static_rigid_body);
}

void World::destroy_object(Object generic) {
  std::visit([this](auto const specific) { destroy_object(specific); }, generic.specific());
}

Particle_data const *World::data(Particle object) const noexcept {
  return _impl->data(object);
}

Particle_data *World::data(Particle object) noexcept {
  return _impl->data(object);
}

Rigid_body_data const *World::data(Rigid_body object) const noexcept {
  return _impl->data(object);
}

Rigid_body_data *World::data(Rigid_body object) noexcept {
  return _impl->data(object);
}

Static_body_data const *World::data(Static_body object) const noexcept {
  return _impl->data(object);
}

Static_body_data *World::data(Static_body object) noexcept {
  return _impl->data(object);
}

World_simulate_result World::simulate(World_simulate_info const &simulate_info) {
  return _impl->simulate(*this, simulate_info);
}
} // namespace physics
} // namespace marlon
