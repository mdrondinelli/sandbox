#include "world.h"

#include <cstdint>

#include <iostream>
#include <latch>

#include "../math/scalar.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "../util/map.h"
#include "aabb_tree.h"
#include "contact.h"
#include "narrowphase.h"

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
    std::uint32_t objects_begin;
    std::uint32_t objects_end;
  };

  template <typename Allocator>
  static std::pair<Block, Neighbor_group_storage>
  make(Allocator &allocator,
       util::Size max_object_count,
       util::Size max_group_count) {
    auto const block =
        allocator.alloc(memory_requirement(max_object_count, max_group_count));
    return {block,
            Neighbor_group_storage{block, max_object_count, max_group_count}};
  }

  static constexpr util::Size memory_requirement(util::Size max_object_count,
                                                 util::Size max_group_count) {
    return Allocator::memory_requirement({
        decltype(_objects)::memory_requirement(max_object_count),
        decltype(_groups)::memory_requirement(max_group_count),
    });
  }

  constexpr Neighbor_group_storage() noexcept = default;

  Neighbor_group_storage(Block block,
                         util::Size max_object_count,
                         util::Size max_group_count)
      : Neighbor_group_storage{block.begin, max_object_count, max_group_count} {
  }

  Neighbor_group_storage(void *block_begin,
                         util::Size max_object_count,
                         util::Size max_group_count) {
    auto allocator = Allocator{make_block(
        block_begin, memory_requirement(max_object_count, max_group_count))};
    _objects = decltype(_objects)::make(allocator, max_object_count).second;
    _groups = decltype(_groups)::make(allocator, max_group_count).second;
  }

  util::Size object_count() const noexcept { return _objects.size(); }

  Object_handle object(util::Size object_index) const noexcept {
    return _objects[object_index];
  }

  util::Size group_count() const noexcept { return _groups.size(); }

  Group const &group(util::Size group_index) const noexcept {
    return _groups[group_index];
  }

  void clear() noexcept {
    _objects.clear();
    _groups.clear();
  }

  void begin_group() {
    auto const objects_index = static_cast<std::uint32_t>(_objects.size());
    _groups.push_back({
        .objects_begin = objects_index,
        .objects_end = objects_index,
    });
  }

  void add_to_group(Particle_handle particle) {
    _objects.push_back(particle.value());
    ++_groups.back().objects_end;
  }

  void add_to_group(Rigid_body_handle rigid_body) {
    _objects.push_back(rigid_body.value());
    ++_groups.back().objects_end;
  }

private:
  List<Object_handle> _objects;
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
    Particle_storage *particles;
    Rigid_body_storage *rigid_bodies;
    Static_body_storage *static_bodies;
    std::latch *latch;
  };

  explicit Narrowphase_task(
      Intrinsic_state const *intrinsic_state,
      Map<std::uint64_t, Contact_manifold>::Iterator first,
      Map<std::uint64_t, Contact_manifold>::Iterator last) noexcept
      : _intrinsic_state{intrinsic_state}, _first{first}, _last{last} {}

  void run(Size) final {
    for (auto it = _first; it != _last; ++it) {
      auto const objects = Object_pair{it->first};
      if (auto const contact = find_contact(objects)) {
        auto object_derived_data = std::array<Object_derived_data, 2>{};
        visit(
            [&](auto &&first_object) {
              object_derived_data[0] = derived_data(data(first_object));
            },
            objects.first());
        visit(
            [&](auto &&second_object) {
              object_derived_data[1] = derived_data(data(second_object));
            },
            objects.second());
        auto const object_positions = std::array<Vec3f, 2>{
            object_derived_data[0].position,
            object_derived_data[1].position,
        };
        auto const object_orientations = std::array<Quatf, 2>{
            object_derived_data[0].orientation,
            object_derived_data[1].orientation,
        };
        it->second.update(object_positions, object_orientations);
        it->second.insert({
            .contact = *contact,
            .initial_object_orientations = object_orientations,
        });
      } else {
        it->second.clear();
      }
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  static_assert(static_cast<int>(Object_type::particle) <
                static_cast<int>(Object_type::rigid_body));
  static_assert(static_cast<int>(Object_type::rigid_body) <
                static_cast<int>(Object_type::static_body));
  std::optional<Contact> find_contact(Object_pair objects) const noexcept {
    auto result = std::optional<Contact>{};
    visit(
        [&](auto &&first_object) {
          using T = std::decay_t<decltype(first_object)>;
          if constexpr (std::is_same_v<T, Particle_handle>) {
            visit(
                [&](auto &&second_object) {
                  result = object_object_contact(
                      {data(first_object), data(second_object)});
                },
                objects.second());
          } else if constexpr (std::is_same_v<T, Rigid_body_handle>) {
            visit(
                [&](auto &&second_object) {
                  using U = std::decay_t<decltype(second_object)>;
                  if constexpr (std::is_same_v<U, Rigid_body_handle> ||
                                std::is_same_v<U, Static_body_handle>) {
                    result = object_object_contact(
                        {data(first_object), data(second_object)});
                  } else {
                    math::unreachable();
                  }
                },
                objects.second());
          } else {
            math::unreachable();
          }
        },
        objects.first());
    return result;
  }

  Object_derived_data derived_data(Particle_data *data) const noexcept {
    return {data->position, Quatf::identity()};
  }

  Object_derived_data derived_data(Rigid_body_data *data) const noexcept {
    return {data->position, data->orientation};
  }

  Object_derived_data derived_data(Static_body_data *data) {
    return {data->position, data->orientation};
  }

  Particle_data *data(Particle_handle object) const noexcept {
    return _intrinsic_state->particles->data(object);
  }

  Rigid_body_data *data(Rigid_body_handle object) const noexcept {
    return _intrinsic_state->rigid_bodies->data(object);
  }

  Static_body_data *data(Static_body_handle object) const noexcept {
    return _intrinsic_state->static_bodies->data(object);
  }

  Intrinsic_state const *_intrinsic_state;
  Map<std::uint64_t, Contact_manifold>::Iterator _first;
  Map<std::uint64_t, Contact_manifold>::Iterator _last;
};

float generalized_inverse_mass(float inverse_mass,
                               Mat3x3f const &inverse_inertia_tensor,
                               Vec3f const &impulse_position,
                               Vec3f const &impulse_direction) noexcept {
  auto const rxn = cross(impulse_position, impulse_direction);
  return inverse_mass + dot(rxn, inverse_inertia_tensor * rxn);
}

class Position_solve_task : public util::Task {
  struct Object_derived_data {
    Mat3x4f transform;
    Mat3x4f inverse_transform;
    float inverse_mass;
    Mat3x3f inverse_inertia_tensor;
  };

public:
  struct Intrinsic_state {
    Particle_storage *particles;
    Rigid_body_storage *rigid_bodies;
    Static_body_storage *static_bodies;
    std::latch *latch;
  };

  struct Work_item {
    Object_pair objects;
    Contact const *contact;
  };

  explicit Position_solve_task(Intrinsic_state const *intrinsic_state,
                               std::span<Work_item const> work_items)
      : _intrinsic_state{intrinsic_state}, _work_items{work_items} {}

  void run(Size /*thread_index*/) final {
    for (auto const &work_item : _work_items) {
      visit(
          [&](auto first_object) {
            visit(
                [&](auto second_object) {
                  auto const object_data =
                      std::pair{data(first_object), data(second_object)};
                  auto const object_derived_data =
                      std::array<Object_derived_data, 2>{
                          derived_data(object_data.first),
                          derived_data(object_data.second),
                      };
                  auto const contact_positions = std::array<Vec3f, 2>{
                      object_derived_data[0].transform *
                          Vec4f{work_item.contact->local_positions[0], 1.0f},
                      object_derived_data[1].transform *
                          Vec4f{work_item.contact->local_positions[1], 1.0f},
                  };
                  auto const relative_position =
                      contact_positions[0] - contact_positions[1];
                  auto const separation =
                      dot(relative_position, work_item.contact->normal) +
                      work_item.contact->separation_bias;
                  if (separation >= 0.0f) {
                    return;
                  }
                  auto const local_contact_normals = std::array<Vec3f, 2>{
                      object_derived_data[0].inverse_transform *
                          Vec4f{work_item.contact->normal, 0.0f},
                      object_derived_data[1].inverse_transform *
                          Vec4f{work_item.contact->normal, 0.0f},
                  };
                  auto const generalized_inverse_masses = std::array<float, 2>{
                      generalized_inverse_mass(
                          object_derived_data[0].inverse_mass,
                          object_derived_data[0].inverse_inertia_tensor,
                          work_item.contact->local_positions[0],
                          local_contact_normals[0]),
                      generalized_inverse_mass(
                          object_derived_data[1].inverse_mass,
                          object_derived_data[1].inverse_inertia_tensor,
                          work_item.contact->local_positions[1],
                          local_contact_normals[1]),
                  };
                  auto const impulse_magnitude =
                      -separation / (generalized_inverse_masses[0] +
                                     generalized_inverse_masses[1]);
                  auto const local_impulses = std::array<Vec3f, 2>{
                      impulse_magnitude * local_contact_normals[0],
                      -impulse_magnitude * local_contact_normals[1],
                  };
                  auto const global_impulse =
                      impulse_magnitude * work_item.contact->normal;
                  apply_impulse(object_data.first,
                                object_derived_data[0],
                                work_item.contact->local_positions[0],
                                local_impulses[0],
                                global_impulse);
                  apply_impulse(object_data.second,
                                object_derived_data[1],
                                work_item.contact->local_positions[1],
                                local_impulses[1],
                                -global_impulse);
                },
                work_item.objects.second());
          },
          work_item.objects.first());
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  void apply_impulse(Particle_data *object_data,
                     Object_derived_data const &derived_data,
                     Vec3f const & /*local_position*/,
                     Vec3f const & /*local_impulse*/,
                     Vec3f const &global_impulse) const noexcept {
    object_data->position += global_impulse * derived_data.inverse_mass;
  }

  void apply_impulse(Rigid_body_data *object_data,
                     Object_derived_data const &derived_data,
                     Vec3f const &local_position,
                     Vec3f const &local_impulse,
                     Vec3f const &global_impulse) const noexcept {
    auto const rotation = Mat3x3f{{derived_data.transform[0][0],
                                   derived_data.transform[0][1],
                                   derived_data.transform[0][2]},
                                  {derived_data.transform[1][0],
                                   derived_data.transform[1][1],
                                   derived_data.transform[1][2]},
                                  {derived_data.transform[2][0],
                                   derived_data.transform[2][1],
                                   derived_data.transform[2][2]}};
    auto const rotated_inverse_inertia_tensor =
        rotation * derived_data.inverse_inertia_tensor;
    object_data->position += global_impulse * derived_data.inverse_mass;
    object_data->orientation +=
        0.5f *
        Quatf{0.0f,
              rotated_inverse_inertia_tensor *
                  cross(local_position, local_impulse)} *
        object_data->orientation;
  }

  void apply_impulse(Static_body_data * /*object_data*/,
                     Object_derived_data const & /*derived_data*/,
                     Vec3f const & /*local_position*/,
                     Vec3f const & /*local_impulse*/,
                     Vec3f const & /*global_impulse*/) const noexcept {}

  Object_derived_data derived_data(Particle_data const *data) const noexcept {
    return Object_derived_data{
        .transform = Mat3x4f::translation(data->position),
        .inverse_transform = Mat3x4f::translation(-data->position),
        .inverse_mass = data->inverse_mass,
        .inverse_inertia_tensor = Mat3x3f::zero(),
    };
  }

  Object_derived_data derived_data(Rigid_body_data const *data) const noexcept {
    auto const transform = Mat3x4f::rigid(data->position, data->orientation);
    return Object_derived_data{
        .transform = transform,
        .inverse_transform = rigid_inverse(transform),
        .inverse_mass = data->inverse_mass,
        .inverse_inertia_tensor = data->inverse_inertia_tensor,
    };
  }

  Object_derived_data
  derived_data(Static_body_data const *data) const noexcept {
    auto const transform = Mat3x4f::rigid(data->position, data->orientation);
    return Object_derived_data{
        .transform = transform,
        .inverse_transform = rigid_inverse(transform),
        .inverse_mass = 0.0f,
        .inverse_inertia_tensor = Mat3x3f::zero(),
    };
  }

  Particle_data *data(Particle_handle object) const noexcept {
    return _intrinsic_state->particles->data(object);
  }

  Rigid_body_data *data(Rigid_body_handle object) const noexcept {
    return _intrinsic_state->rigid_bodies->data(object);
  }

  Static_body_data *data(Static_body_handle object) const noexcept {
    return _intrinsic_state->static_bodies->data(object);
  }

  Intrinsic_state const *_intrinsic_state;
  std::span<Work_item const> _work_items;
};

class Velocity_solve_task : public util::Task {
  struct Object_derived_data {
    Vec3f velocity;
    Vec3f angular_velocity;
    Mat3x3f rotation;
    Mat3x4f inverse_transform;
    float inverse_mass;
    Mat3x3f inverse_inertia_tensor;
    Material material;
  };

public:
  struct Intrinsic_state {
    Particle_storage *particles;
    Rigid_body_storage *rigid_bodies;
    Static_body_storage *static_bodies;
    std::latch *latch;
    float restitution_separating_velocity_epsilon;
  };

  struct Work_item {
    Object_pair objects;
    Contact const *contact;
  };

  explicit Velocity_solve_task(Intrinsic_state const *intrinsic_state,
                               std::span<Work_item const> work_items)
      : _intrinsic_state{intrinsic_state}, _work_items{work_items} {}

  void run(Size /*thread_index*/) final {
    for (auto const &work_item : _work_items) {
      visit(
          [&](auto first_object) {
            visit(
                [&](auto second_object) {
                  auto const object_data =
                      std::pair{data(first_object), data(second_object)};
                  auto const object_derived_data =
                      std::array<Object_derived_data, 2>{
                          derived_data(object_data.first),
                          derived_data(object_data.second),
                      };
                  auto const relative_contact_positions = std::array<Vec3f, 2>{
                      object_derived_data[0].rotation *
                          work_item.contact->local_positions[0],
                      object_derived_data[1].rotation *
                          work_item.contact->local_positions[1],
                  };
                  auto const relative_velocity =
                      (object_derived_data[0].velocity +
                       cross(object_derived_data[0].angular_velocity,
                             relative_contact_positions[0])) -
                      (object_derived_data[1].velocity +
                       cross(object_derived_data[1].angular_velocity,
                             relative_contact_positions[1]));
                  auto const separating_velocity =
                      dot(relative_velocity, work_item.contact->normal);
                  if (separating_velocity >= 0.0f) {
                    return;
                  }
                  auto const local_contact_normals = std::array<Vec3f, 2>{
                      object_derived_data[0].inverse_transform *
                          Vec4f{work_item.contact->normal, 0.0f},
                      object_derived_data[1].inverse_transform *
                          Vec4f{work_item.contact->normal, 0.0f},
                  };
                  auto const normal_generalized_inverse_masses =
                      std::array<float, 2>{
                          generalized_inverse_mass(
                              object_derived_data[0].inverse_mass,
                              object_derived_data[0].inverse_inertia_tensor,
                              work_item.contact->local_positions[0],
                              local_contact_normals[0]),
                          generalized_inverse_mass(
                              object_derived_data[1].inverse_mass,
                              object_derived_data[1].inverse_inertia_tensor,
                              work_item.contact->local_positions[1],
                              local_contact_normals[1]),
                      };
                  auto const restitution_coefficient =
                      -separating_velocity >
                              _intrinsic_state
                                  ->restitution_separating_velocity_epsilon
                          ? 0.5f * (object_derived_data[0]
                                        .material.restitution_coefficient +
                                    object_derived_data[1]
                                        .material.restitution_coefficient)
                          : 0.0f;
                  auto const normal_impulse_magnitude =
                      (-separating_velocity *
                       (1.0f + restitution_coefficient)) /
                      (normal_generalized_inverse_masses[0] +
                       normal_generalized_inverse_masses[1]);
                  auto local_impulses = std::array<Vec3f, 2>{
                      normal_impulse_magnitude * local_contact_normals[0],
                      -normal_impulse_magnitude * local_contact_normals[1],
                  };
                  auto global_impulse =
                      normal_impulse_magnitude * work_item.contact->normal;
                  auto const tangential_velocity =
                      relative_velocity -
                      separating_velocity * work_item.contact->normal;
                  if (tangential_velocity != Vec3f::zero()) {
                    auto const tangential_speed = length(tangential_velocity);
                    auto const global_tangent =
                        tangential_velocity / tangential_speed;
                    auto const local_tangents = std::array<Vec3f, 2>{
                        object_derived_data[0].inverse_transform *
                            Vec4f{global_tangent, 0.0f},
                        object_derived_data[1].inverse_transform *
                            Vec4f{global_tangent, 0.0f},
                    };
                    auto const tangential_generalized_inverse_masses =
                        std::array<float, 2>{
                            generalized_inverse_mass(
                                object_derived_data[0].inverse_mass,
                                object_derived_data[0].inverse_inertia_tensor,
                                work_item.contact->local_positions[0],
                                -local_tangents[0]),
                            generalized_inverse_mass(
                                object_derived_data[1].inverse_mass,
                                object_derived_data[1].inverse_inertia_tensor,
                                work_item.contact->local_positions[1],
                                -local_tangents[1]),
                        };
                    auto const inverse_sum_tangential_generalized_inverse_mass =
                        1.0f / (tangential_generalized_inverse_masses[0] +
                                tangential_generalized_inverse_masses[1]);
                    auto const static_friction_impulse_magnitude =
                        tangential_speed *
                        inverse_sum_tangential_generalized_inverse_mass;
                    auto const static_friction_coefficient =
                        0.5f * (object_derived_data[0]
                                    .material.static_friction_coefficient +
                                object_derived_data[1]
                                    .material.dynamic_friction_coefficient);
                    if (static_friction_impulse_magnitude <=
                        static_friction_coefficient *
                            normal_impulse_magnitude) {
                      local_impulses[0] -=
                          static_friction_impulse_magnitude * local_tangents[0];
                      local_impulses[1] +=
                          static_friction_impulse_magnitude * local_tangents[1];
                      global_impulse -=
                          static_friction_impulse_magnitude * global_tangent;
                    } else {
                      auto const dynamic_friction_coefficient =
                          0.5f * (object_derived_data[0]
                                      .material.dynamic_friction_coefficient +
                                  object_derived_data[1]
                                      .material.dynamic_friction_coefficient);
                      auto const dynamic_friction_impulse_magnitude =
                          dynamic_friction_coefficient *
                          normal_impulse_magnitude;
                      local_impulses[0] -= dynamic_friction_impulse_magnitude *
                                           local_tangents[0];
                      local_impulses[1] += dynamic_friction_impulse_magnitude *
                                           local_tangents[1];
                      global_impulse -=
                          dynamic_friction_impulse_magnitude * global_tangent;
                    }
                  }
                  apply_impulse(object_data.first,
                                object_derived_data[0],
                                work_item.contact->local_positions[0],
                                local_impulses[0],
                                global_impulse);
                  apply_impulse(object_data.second,
                                object_derived_data[1],
                                work_item.contact->local_positions[1],
                                local_impulses[1],
                                -global_impulse);
                },
                work_item.objects.second());
          },
          work_item.objects.first());
    }
    if (auto const latch = _intrinsic_state->latch) {
      latch->count_down();
    }
  }

private:
  void apply_impulse(Particle_data *object_data,
                     Object_derived_data const &derived_data,
                     Vec3f const & /*local_position*/,
                     Vec3f const & /*local_impulse*/,
                     Vec3f const &global_impulse) const noexcept {
    object_data->velocity += global_impulse * derived_data.inverse_mass;
  }

  void apply_impulse(Rigid_body_data *object_data,
                     Object_derived_data const &derived_data,
                     Vec3f const &local_position,
                     Vec3f const &local_impulse,
                     Vec3f const &global_impulse) const noexcept {
    auto const rotated_inverse_inertia_tensor =
        derived_data.rotation * derived_data.inverse_inertia_tensor;
    object_data->velocity += global_impulse * derived_data.inverse_mass;
    object_data->angular_velocity +=
        rotated_inverse_inertia_tensor * cross(local_position, local_impulse);
  }

  void apply_impulse(Static_body_data * /*object_data*/,
                     Object_derived_data const & /*derived_data*/,
                     Vec3f const & /*local_position*/,
                     Vec3f const & /*local_impulse*/,
                     Vec3f const & /*global_impulse*/) const noexcept {}

  Object_derived_data derived_data(Particle_data const *data) const noexcept {
    return Object_derived_data{
        .velocity = data->velocity,
        .angular_velocity = Vec3f::zero(),
        .rotation = Mat3x3f::identity(),
        .inverse_transform = Mat3x4f::translation(-data->position),
        .inverse_mass = data->inverse_mass,
        .inverse_inertia_tensor = Mat3x3f::zero(),
        .material = data->material,
    };
  }

  Object_derived_data derived_data(Rigid_body_data const *data) const noexcept {
    auto const transform = Mat3x4f::rigid(data->position, data->orientation);
    return Object_derived_data{
        .velocity = data->velocity,
        .angular_velocity = data->angular_velocity,
        .rotation =
            Mat3x3f{{transform[0][0], transform[0][1], transform[0][2]},
                    {transform[1][0], transform[1][1], transform[1][2]},
                    {transform[2][0], transform[2][1], transform[2][2]}},
        .inverse_transform = rigid_inverse(transform),
        .inverse_mass = data->inverse_mass,
        .inverse_inertia_tensor = data->inverse_inertia_tensor,
        .material = data->material,
    };
  }

  Object_derived_data
  derived_data(Static_body_data const *data) const noexcept {
    auto const transform = Mat3x4f::rigid(data->position, data->orientation);
    return Object_derived_data{
        .velocity = Vec3f::zero(),
        .angular_velocity = Vec3f::zero(),
        .rotation =
            Mat3x3f{{transform[0][0], transform[0][1], transform[0][2]},
                    {transform[1][0], transform[1][1], transform[1][2]},
                    {transform[2][0], transform[2][1], transform[2][2]}},
        .inverse_transform = rigid_inverse(transform),
        .inverse_mass = 0.0f,
        .inverse_inertia_tensor = Mat3x3f::zero(),
        .material = data->material,
    };
  }

  Particle_data *data(Particle_handle object) const noexcept {
    return _intrinsic_state->particles->data(object);
  }

  Rigid_body_data *data(Rigid_body_handle object) const noexcept {
    return _intrinsic_state->rigid_bodies->data(object);
  }

  Static_body_data *data(Static_body_handle object) const noexcept {
    return _intrinsic_state->static_bodies->data(object);
  }

  Intrinsic_state const *_intrinsic_state;
  std::span<Work_item const> _work_items;
};

// integration constants
auto constexpr velocity_damping_factor = 0.99f;
auto constexpr waking_motion_epsilon = 0.01f;
auto constexpr waking_motion_initializer = 2.0f * waking_motion_epsilon;
auto constexpr waking_motion_limit = 10.0f * waking_motion_epsilon;
auto constexpr waking_motion_smoothing_factor = 0.8f;
auto constexpr max_narrowphase_task_size = Size{16};
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
            // create_info.max_neighbor_pairs,
            create_info.max_neighbor_groups),
        decltype(_neighbor_group_awake_indices)::memory_requirement(
            create_info.max_neighbor_groups),
        // decltype(_coloring_bits)::memory_requirement(max_colors),
        // decltype(_coloring_fringe)::memory_requirement(
        //     create_info.max_neighbor_pairs),
        // decltype(_color_groups)::memory_requirement(
        //     create_info.max_neighbor_pairs),
        decltype(_contact_manifolds)::memory_requirement(
            create_info.max_neighbor_pairs),
        decltype(_narrowphase_tasks)::memory_requirement(
            (create_info.max_neighbor_pairs + max_narrowphase_task_size - 1) /
            max_narrowphase_task_size),
    });
  }

  explicit Impl(World_create_info const &create_info)
      : _threads{create_info.worker_thread_count},
        _block{util::System_allocator::instance()->alloc(
            memory_requirement(create_info))},
        _gravitational_acceleration{create_info.gravitational_acceleration} {
    auto allocator = Stack_allocator<>{_block};
    _particles =
        Particle_storage::make(allocator, create_info.max_particles).second;
    _rigid_bodies =
        Rigid_body_storage::make(allocator, create_info.max_rigid_bodies)
            .second;
    _static_bodies =
        Static_body_storage::make(allocator, create_info.max_static_bodies)
            .second;
    _aabb_tree =
        Aabb_tree<Object_handle>::make(allocator,
                                       create_info.max_aabb_tree_leaf_nodes,
                                       create_info.max_aabb_tree_internal_nodes)
            .second;
    _neighbor_pairs =
        List<Object_pair>::make(allocator, create_info.max_neighbor_pairs)
            .second;
    _neighbor_pair_ptrs =
        List<Object_pair *>::make(allocator, 2 * create_info.max_neighbor_pairs)
            .second;
    _neighbor_groups =
        Neighbor_group_storage::make(allocator,
                                     create_info.max_particles +
                                         create_info.max_rigid_bodies,
                                     //  create_info.max_neighbor_pairs,
                                     create_info.max_neighbor_groups)
            .second;
    _neighbor_group_awake_indices =
        List<std::uint32_t>::make(allocator, create_info.max_neighbor_groups)
            .second;
    // _coloring_bits = Bit_list::make(allocator, max_colors).second;
    // _coloring_bits.resize(max_colors);
    // _coloring_fringe =
    //     Queue<Object_pair *>::make(allocator,
    //     create_info.max_neighbor_pairs)
    //         .second;
    // _color_groups =
    //     Color_group_storage::make(allocator, create_info.max_neighbor_pairs)
    //         .second;
    _contact_manifolds = Map<std::uint64_t, Contact_manifold>::make(
                             allocator, create_info.max_neighbor_pairs)
                             .second;
    _narrowphase_tasks =
        List<Narrowphase_task>::make(
            allocator,
            (create_info.max_neighbor_pairs + max_narrowphase_task_size - 1) /
                max_narrowphase_task_size)
            .second;
  }

  ~Impl() {
    _contact_manifolds = {};
    // _color_groups = {};
    // _coloring_fringe = {};
    // _coloring_bits = {};
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
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Object_handle{}),
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
    _particles.data(particle)->aabb_tree_node->payload = particle.value();
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
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Object_handle{}),
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
    _rigid_bodies.data(rigid_body)->aabb_tree_node->payload =
        rigid_body.value();
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
    // auto const transform_inverse = rigid_inverse(transform);
    auto const bounds = physics::bounds(create_info.shape, transform);
    auto const static_body = _static_bodies.create({
        .aabb_tree_node = _aabb_tree.create_leaf(bounds, Object_handle{}),
        .shape = create_info.shape,
        .material = create_info.material,
        .position = create_info.position,
        .orientation = create_info.orientation,
    });
    _static_bodies.data(static_body)->aabb_tree_node->payload =
        static_body.value();
    return static_body;
  }

  void destroy_static_body(Static_body_handle handle) {
    _aabb_tree.destroy_leaf(_static_bodies.data(handle)->aabb_tree_node);
    _static_bodies.destroy(handle);
  }

  void simulate(World const &world, World_simulate_info const &simulate_info) {
    // _threads.set_scheduling_policy(util::Scheduling_policy::spin);
    build_aabb_tree(simulate_info.delta_time);
    clear_neighbor_pairs();
    find_neighbor_pairs();
    assign_neighbor_pairs();
    find_neighbor_groups();
    _neighbor_group_awake_indices.clear();
    // _color_groups.clear();
    for (auto j = util::Size{}; j != _neighbor_groups.group_count(); ++j) {
      if (update_neighbor_group_awake_states(j)) {
        _neighbor_group_awake_indices.emplace_back(j);
        // color_neighbor_group(j);
      }
    }
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
    auto const time_compensated_velocity_damping_factor =
        pow(velocity_damping_factor, h);
    auto const time_compensating_waking_motion_smoothing_factor =
        1.0f - pow(1.0f - waking_motion_smoothing_factor, h);
    auto const restitution_separating_velocity_epsilon =
        2.0f * h * length(_gravitational_acceleration);
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      integrate(h,
                time_compensated_velocity_damping_factor,
                time_compensating_waking_motion_smoothing_factor);
      find_contacts();
      solve_positions();
      solve_velocities(restitution_separating_velocity_epsilon);
    }
    // _threads.set_scheduling_policy(util::Scheduling_policy::block);
    call_motion_callbacks(world);
  }

private:
  void build_aabb_tree(float delta_time) {
    auto const constant_safety_term = 0.0f;
    auto const velocity_safety_factor = 2.0f;
    auto const gravity_safety_factor = 2.0f;
    auto const gravity_safety_term = gravity_safety_factor *
                                     length(_gravitational_acceleration) *
                                     delta_time * delta_time;
    _particles.for_each([&](Particle_handle object) {
      auto const object_data = get_data(object);
      auto const half_extents = Vec3f::all(
          object_data->radius + constant_safety_term +
          velocity_safety_factor * length(object_data->velocity) * delta_time +
          gravity_safety_term);
      object_data->aabb_tree_node->bounds = {
          object_data->position - half_extents,
          object_data->position + half_extents};
    });
    _rigid_bodies.for_each([&](Rigid_body_handle object) {
      auto const object_data = get_data(object);
      object_data->aabb_tree_node->bounds =
          expand(bounds(object_data->shape,
                        Mat3x4f::rigid(object_data->position,
                                       object_data->orientation)),
                 constant_safety_term +
                     velocity_safety_factor * length(object_data->velocity) *
                         delta_time +
                     gravity_safety_term);
    });
    _aabb_tree.build();
  }

  void clear_neighbor_pairs() {
    auto const reset_neighbor_count = [&](auto object) {
      get_data(object)->neighbor_count = 0;
    };
    _particles.for_each(reset_neighbor_count);
    _rigid_bodies.for_each(reset_neighbor_count);
    _neighbor_pair_ptrs.clear();
    _neighbor_pairs.clear();
    _neighbor_groups.clear();
  }

  void find_neighbor_pairs() {
    _aabb_tree.for_each_overlapping_leaf_pair(
        [this](Object_handle first_payload, Object_handle second_payload) {
          visit(
              [&](auto &&first_handle) {
                visit(
                    [&](auto &&second_handle) {
                      using T = std::decay_t<decltype(first_handle)>;
                      using U = std::decay_t<decltype(second_handle)>;
                      if constexpr (!std::is_same_v<T, Static_body_handle> ||
                                    !std::is_same_v<U, Static_body_handle>) {
                        _neighbor_pairs.emplace_back(
                            std::pair{first_payload, second_payload});
                        auto const id = _neighbor_pairs.back().id();
                        auto const it = _contact_manifolds
                                            .emplace(std::piecewise_construct,
                                                     std::tuple{id},
                                                     std::tuple{})
                                            .first;
                        it->second.marked(true);
                        increment_neighbor_count(get_data(first_handle));
                        increment_neighbor_count(get_data(second_handle));
                      }
                    },
                    second_payload);
              },
              first_payload);
        });
    for (auto it = _contact_manifolds.begin();
         it != _contact_manifolds.end();) {
      if (it->second.marked()) {
        it->second.marked(false);
        ++it;
      } else {
        it = _contact_manifolds.erase(it);
      }
    }
  }

  void increment_neighbor_count(Particle_data *data) noexcept {
    ++data->neighbor_count;
  }

  void increment_neighbor_count(Rigid_body_data *data) noexcept {
    ++data->neighbor_count;
  }

  void increment_neighbor_count(Static_body_data *) const noexcept {}

  void assign_neighbor_pairs() {
    auto const alloc_neighbor_pairs = [this](auto object) {
      auto const object_data = get_data(object);
      object_data->neighbor_pairs = _neighbor_pair_ptrs.end();
      _neighbor_pair_ptrs.resize(_neighbor_pair_ptrs.size() +
                                 object_data->neighbor_count);
      object_data->neighbor_count = 0;
    };
    _particles.for_each(alloc_neighbor_pairs);
    _rigid_bodies.for_each(alloc_neighbor_pairs);
    for (auto &pair : _neighbor_pairs) {
      visit(
          [&](auto &&handle) { assign_neighbor_pair(get_data(handle), &pair); },
          pair.first());
      visit(
          [&](auto &&handle) { assign_neighbor_pair(get_data(handle), &pair); },
          pair.second());
    }
  }

  void assign_neighbor_pair(Particle_data *data,
                            Object_pair *neighbor_pair) noexcept {
    data->neighbor_pairs[data->neighbor_count++] = neighbor_pair;
  }

  void assign_neighbor_pair(Rigid_body_data *data,
                            Object_pair *neighbor_pair) noexcept {
    data->neighbor_pairs[data->neighbor_count++] = neighbor_pair;
  }

  void assign_neighbor_pair(Static_body_data *, Object_pair *) const noexcept {}

  void find_neighbor_groups() {
    auto const unmark = [&](auto object) { get_data(object)->marked = false; };
    _particles.for_each(unmark);
    _rigid_bodies.for_each(unmark);
    auto const visitor = [this](auto &&handle) {
      using T = std::decay_t<decltype(handle)>;
      if constexpr (!std::is_same_v<T, Static_body_handle>) {
        for (auto const pair : get_neighbor_pairs(get_data(handle))) {
          visit(
              [&](auto &&neighbor_handle) {
                using U = std::decay_t<decltype(neighbor_handle)>;
                if constexpr (!std::is_same_v<U, Static_body_handle>) {
                  auto const neighbor_data = get_data(neighbor_handle);
                  if (!neighbor_data->marked) {
                    neighbor_data->marked = true;
                    _neighbor_groups.add_to_group(neighbor_handle);
                  }
                  // if (pair->color() == color_unmarked) {
                  //   pair->color(color_marked);
                  //   _neighbor_groups.add_to_group(pair);
                  // }
                } else {
                  // _neighbor_groups.add_to_group(pair);
                }
              },
              pair->other(handle.value()));
        }
      } else {
        math::unreachable();
      }
    };
    auto fringe_index = util::Size{};
    auto const find_neighbor_group = [&, this](auto const seed_object) {
      auto const seed_data = get_data(seed_object);
      if (!seed_data->marked) {
        seed_data->marked = true;
        _neighbor_groups.begin_group();
        _neighbor_groups.add_to_group(seed_object);
        do {
          visit(visitor, _neighbor_groups.object(fringe_index));
        } while (++fringe_index != _neighbor_groups.object_count());
      }
    };
    _particles.for_each(find_neighbor_group);
    _rigid_bodies.for_each(find_neighbor_group);
  }

  bool update_neighbor_group_awake_states(util::Size group_index) {
    auto const &group = _neighbor_groups.group(group_index);
    auto contains_awake = false;
    auto contains_sleeping = false;
    auto sleepable = true;
    for (auto i = group.objects_begin;
         (sleepable || !contains_awake || !contains_sleeping) &&
         i != group.objects_end;
         ++i) {
      visit(
          [&](auto &&handle) {
            using T = std::decay_t<decltype(handle)>;
            if constexpr (!std::is_same_v<T, Static_body_handle>) {
              auto const data = get_data(handle);
              if (data->awake) {
                contains_awake = true;
                if (data->waking_motion > waking_motion_epsilon) {
                  sleepable = false;
                }
              } else {
                contains_sleeping = true;
              }
            } else {
              math::unreachable();
            }
          },
          _neighbor_groups.object(i));
    }
    if (contains_awake) {
      if (sleepable) {
        for (auto i = group.objects_begin; i != group.objects_end; ++i) {
          auto const object = _neighbor_groups.object(i);
          visit(
              [&](auto &&handle) {
                using T = std::decay_t<decltype(handle)>;
                if constexpr (!std::is_same_v<T, Static_body_handle>) {
                  auto const data = get_data(handle);
                  if (data->awake) {
                    data->awake = false;
                    freeze(data);
                  }
                } else {
                  math::unreachable();
                }
              },
              object);
        }
        return false;
      } else {
        if (contains_sleeping) {
          for (auto i = group.objects_begin; i != group.objects_end; ++i) {
            visit(
                [&](auto &&handle) {
                  using T = std::decay_t<decltype(handle)>;
                  if constexpr (!std::is_same_v<T, Static_body_handle>) {
                    auto const data = get_data(handle);
                    if (!data->awake) {
                      data->awake = true;
                      data->waking_motion = waking_motion_initializer;
                    }
                  } else {
                    math::unreachable();
                  }
                },
                _neighbor_groups.object(i));
          }
        }
        return true;
      }
    } else {
      return false;
    }
  }

  void freeze(Particle_data *data) { data->velocity = Vec3f::zero(); }

  void freeze(Rigid_body_data *data) {
    data->velocity = Vec3f::zero();
    data->angular_velocity = Vec3f::zero();
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
  //   for (auto const i : _neighbor_group_awake_indices) {
  //     auto const &group = _neighbor_groups.group(i);
  //     auto const begin = group.neighbor_pairs_begin;
  //     auto const end = group.neighbor_pairs_end;
  //     for (auto j = begin; j != end; ++j) {
  //       _color_groups.push_back(_neighbor_groups.neighbor_pair(j));
  //     }
  //   }
  // }

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

  void integrate_neighbor_group(util::Size group_index,
                                float delta_time,
                                float velocity_damping_factor,
                                float waking_motion_smoothing_factor) {
    auto const &group = _neighbor_groups.group(group_index);
    for (auto i = group.objects_begin; i != group.objects_end; ++i) {
      visit(
          [&](auto &&object) {
            using T = std::decay_t<decltype(object)>;
            if constexpr (!std::is_same_v<T, Static_body_handle>) {
              integrate(get_data(object),
                        delta_time,
                        velocity_damping_factor,
                        waking_motion_smoothing_factor);
            } else {
              math::unreachable();
            }
          },
          _neighbor_groups.object(i));
    }
  }

  void integrate(Particle_data *particle,
                 float delta_time,
                 float velocity_damping_factor,
                 float waking_motion_smoothing_factor) {
    particle->previous_position = particle->position;
    particle->velocity += delta_time * _gravitational_acceleration;
    particle->velocity *= velocity_damping_factor;
    particle->position += delta_time * particle->velocity;
    particle->waking_motion = min(
        (1.0f - waking_motion_smoothing_factor) * particle->waking_motion +
            waking_motion_smoothing_factor * length_squared(particle->velocity),
        waking_motion_limit);
  }

  void integrate(Rigid_body_data *rigid_body,
                 float delta_time,
                 float velocity_damping_factor,
                 float waking_motion_smoothing_factor) {
    rigid_body->previous_position = rigid_body->position;
    rigid_body->previous_orientation = rigid_body->orientation;
    rigid_body->velocity += delta_time * _gravitational_acceleration;
    rigid_body->velocity *= velocity_damping_factor;
    rigid_body->position += delta_time * rigid_body->velocity;
    rigid_body->angular_velocity *= velocity_damping_factor;
    rigid_body->orientation +=
        Quatf{0.0f, 0.5f * delta_time * rigid_body->angular_velocity} *
        rigid_body->orientation;
    rigid_body->orientation = normalize(rigid_body->orientation);
    rigid_body->waking_motion = min(
        (1.0f - waking_motion_smoothing_factor) * rigid_body->waking_motion +
            waking_motion_smoothing_factor *
                (length_squared(rigid_body->velocity) +
                 length_squared(rigid_body->angular_velocity)),
        waking_motion_limit);
  }

  void find_contacts() {
    auto const complete_task_count =
        _contact_manifolds.size() / max_narrowphase_task_size;
    auto const partial_task_size =
        _contact_manifolds.size() % max_narrowphase_task_size;
    auto latch =
        std::latch{complete_task_count + (partial_task_size > 0 ? 1 : 0)};
    auto const intrinsic_state = Narrowphase_task::Intrinsic_state{
        .particles = &_particles,
        .rigid_bodies = &_rigid_bodies,
        .static_bodies = &_static_bodies,
        .latch = !_threads.empty() ? &latch : nullptr,
    };
    _narrowphase_tasks.clear();
    auto it = _contact_manifolds.begin();
    for (auto i = Size{}; i < complete_task_count; ++i) {
      auto const first = it;
      for (auto j = Size{}; j < max_narrowphase_task_size; ++j) {
        ++it;
      }
      auto const last = it;
      auto &task =
          _narrowphase_tasks.emplace_back(&intrinsic_state, first, last);
      if (!_threads.empty()) {
        _threads.push_silent(&task);
      } else {
        task.run({});
      }
    }
    if (partial_task_size > 0) {
      auto const first = it;
      for (auto i = Size{}; i < partial_task_size; ++i) {
        ++it;
      }
      auto const last = it;
      auto &task =
          _narrowphase_tasks.emplace_back(&intrinsic_state, first, last);
      if (!_threads.empty()) {
        _threads.push_silent(&task);
      } else {
        task.run({});
      }
    }
    if (!_threads.empty()) {
      _threads.notify();
      for (;;) {
        if (latch.try_wait()) {
          return;
        }
      }
      // latch.wait();
    }
  }

  void solve_positions() {
    auto const intrinsic_state = Position_solve_task::Intrinsic_state{
        .particles = &_particles,
        .rigid_bodies = &_rigid_bodies,
        .static_bodies = &_static_bodies,
        .latch = nullptr,
    };
    for (auto i = 0; i != 2; ++i) {
      for (auto &[id, contact_manifold] : _contact_manifolds) {
        for (auto const &contact : contact_manifold.contacts()) {
          auto const work_item = Position_solve_task::Work_item{
              .objects = Object_pair{id},
              .contact = &contact.contact,
          };
          Position_solve_task{&intrinsic_state, {&work_item, 1u}}.run({});
        }
      }
    }
  }

  void solve_velocities(float restitution_separating_velocity_epsilon) {
    auto const intrinsic_state = Velocity_solve_task::Intrinsic_state{
        .particles = &_particles,
        .rigid_bodies = &_rigid_bodies,
        .static_bodies = &_static_bodies,
        .latch = nullptr,
        .restitution_separating_velocity_epsilon =
            restitution_separating_velocity_epsilon,
    };
    for (auto i = 0; i != 2; ++i) {
      for (auto &[id, contact_manifold] : _contact_manifolds) {
        for (auto const &contact : contact_manifold.contacts()) {
          auto const work_item = Velocity_solve_task::Work_item{
              .objects = Object_pair{id},
              .contact = &contact.contact,
          };
          Velocity_solve_task{&intrinsic_state, {&work_item, 1u}}.run({});
        }
      }
    }
    // auto solve_chunk_index = std::size_t{};
    // for (auto j = std::size_t{}; j != max_colors; ++j) {
    //   auto const color = static_cast<std::uint16_t>(j);
    //   auto const group = _color_groups.group(color);
    //   if (!group.empty()) {
    //     auto const solve_chunk_count =
    //         (group.size() + max_solve_chunk_size - 1) / max_solve_chunk_size;
    //     auto latch =
    //     std::latch{static_cast<std::ptrdiff_t>(solve_chunk_count)};
    //     solve_state.latch = &latch;
    //     for (auto k = std::size_t{}; k != solve_chunk_count; ++k) {
    //       thread_pool.push(&_velocity_solve_tasks[solve_chunk_index + k]);
    //     }
    //     for (;;) {
    //       if (latch.try_wait()) {
    //         break;
    //       }
    //     }
    //     solve_chunk_index += solve_chunk_count;
    //   } else {
    //     break;
    //   }
    // }
  }

  void call_motion_callbacks(World const &world) {
    auto call_object_motion_callback = [&](auto object) {
      using T = decltype(object);
      auto const object_data = get_data(object);
      if (object_data->awake && object_data->motion_callback != nullptr) {
        if constexpr (std::is_same_v<T, Particle_handle>) {
          object_data->motion_callback->on_particle_motion(world, object);
        } else {
          static_assert(std::is_same_v<T, Rigid_body_handle>);
          object_data->motion_callback->on_rigid_body_motion(world, object);
        }
      }
    };
    _particles.for_each(call_object_motion_callback);
    _rigid_bodies.for_each(call_object_motion_callback);
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

  std::span<Object_pair *const>
  get_neighbor_pairs(Particle_data *particle) const noexcept {
    return {particle->neighbor_pairs, particle->neighbor_count};
  }

  std::span<Object_pair *const>
  get_neighbor_pairs(Rigid_body_data *rigid_body) const noexcept {
    return {rigid_body->neighbor_pairs, rigid_body->neighbor_count};
  }

  std::span<Object_pair *const>
  get_neighbor_pairs(Static_body_data *) const noexcept {
    return {};
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

  Thread_pool _threads;
  Block _block;
  Particle_storage _particles;
  Static_body_storage _static_bodies;
  Rigid_body_storage _rigid_bodies;
  Aabb_tree<Object_handle> _aabb_tree;
  List<Object_pair> _neighbor_pairs;
  List<Object_pair *> _neighbor_pair_ptrs;
  Neighbor_group_storage _neighbor_groups;
  List<std::uint32_t> _neighbor_group_awake_indices;
  // Bit_list _coloring_bits;
  // Queue<Object_pair *> _coloring_fringe;
  // Color_group_storage _color_groups;
  // List<Contact> _contacts;
  Map<std::uint64_t, Contact_manifold> _contact_manifolds;
  List<Narrowphase_task> _narrowphase_tasks;
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