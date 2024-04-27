#ifndef MARLON_PHYSICS_RIGID_BODY_H
#define MARLON_PHYSICS_RIGID_BODY_H

#include <cstdint>

#include "../math/math.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "material.h"
#include "object.h"
#include "shape.h"

namespace marlon {
namespace physics {
class Rigid_body_motion_callback;

struct Rigid_body_data {
  Aabb_tree<Object_handle>::Node *aabb_tree_node{};
  Object_pair **neighbor_pairs{};
  Rigid_body_motion_callback *motion_callback{};
  Shape shape;
  float inverse_mass{};
  math::Mat3x3f inverse_inertia_tensor{};
  Material material{};
  math::Vec3f previous_position{};
  math::Vec3f position{};
  math::Vec3f velocity{};
  math::Quatf previous_orientation{};
  math::Quatf orientation{};
  math::Vec3f angular_velocity{};
  float waking_motion;
  std::uint16_t neighbor_count{};
  bool marked{};
  bool awake{};
};

class World;

class Rigid_body_motion_callback {
public:
  virtual ~Rigid_body_motion_callback() = default;

  virtual void on_rigid_body_motion(World const &world,
                                    Rigid_body_handle rigid_body) = 0;
};

class Rigid_body_storage {
  using Allocator = util::Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<util::Block, Rigid_body_storage>
  make(Allocator &allocator, util::Size max_rigid_bodies) {
    auto const block = allocator.alloc(memory_requirement(max_rigid_bodies));
    return {block, Rigid_body_storage{block, max_rigid_bodies}};
  }

  static constexpr util::Size
  memory_requirement(util::Size max_rigid_bodies) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_rigid_bodies),
        decltype(_available_handles)::memory_requirement(max_rigid_bodies),
        decltype(_occupancy_bits)::memory_requirement(max_rigid_bodies),
    });
  }

  constexpr Rigid_body_storage() = default;

  explicit Rigid_body_storage(util::Block block,
                              util::Size max_rigid_bodies) noexcept
      : Rigid_body_storage{block.begin, max_rigid_bodies} {}

  explicit Rigid_body_storage(void *block,
                              util::Size max_rigid_bodies) noexcept {
    auto allocator = Allocator{
        util::make_block(block, memory_requirement(max_rigid_bodies))};
    _data = decltype(_data)::make(allocator, max_rigid_bodies).second;
    _data.resize(max_rigid_bodies);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_rigid_bodies).second;
    _available_handles.resize(max_rigid_bodies);
    for (auto i = util::Size{}; i != max_rigid_bodies; ++i) {
      _available_handles[i] = Rigid_body_handle{
          static_cast<Object_handle>(max_rigid_bodies - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_rigid_bodies).second;
    _occupancy_bits.resize(max_rigid_bodies);
  }

  Rigid_body_handle create(Rigid_body_data const &data) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{
          "Capacity_error in Rigid_body_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(data);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Rigid_body_handle rigid_body) {
    _available_handles.emplace_back(rigid_body);
    _occupancy_bits.reset(rigid_body.index());
  }

  Rigid_body_data const *data(Rigid_body_handle rigid_body) const noexcept {
    return _data[rigid_body.index()].get();
  }

  Rigid_body_data *data(Rigid_body_handle rigid_body) noexcept {
    return _data[rigid_body.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = util::Size{};
    for (auto i = util::Size{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        f(Rigid_body_handle{static_cast<Object_handle>(i)});
        ++k;
      }
    }
  }

private:
  util::List<util::Lifetime_box<Rigid_body_data>> _data;
  util::List<Rigid_body_handle> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif