#ifndef MARLON_PHYSICS_STATIC_BODY_H
#define MARLON_PHYSICS_STATIC_BODY_H

#include <bitset>

#include "../math/math.h"
#include "material.h"
#include "object.h"
#include "shape.h"

namespace marlon {
namespace physics {
class Static_body_data {
public:
  explicit Static_body_data(Broadphase_bvh::Node *bvh_node,
                            math::Vec3f const &position,
                            math::Quatf const &orientation,
                            Shape const &shape,
                            Material const &material) noexcept
      : _bvh_node{bvh_node},
        _position{position},
        _orientation{orientation},
        _shape{shape},
        _material{material} {}

  Broadphase_bvh::Node const *bvh_node() const noexcept { return _bvh_node; }

  Broadphase_bvh::Node *bvh_node() noexcept { return _bvh_node; }

  math::Vec3f const &position() const noexcept { return _position; }

  math::Quatf const &orientation() const noexcept { return _orientation; }

  Shape const &shape() const noexcept { return _shape; }

  Material const &material() const noexcept { return _material; }

private:
  Broadphase_bvh::Node *_bvh_node;
  math::Vec3f _position;
  math::Quatf _orientation;
  Shape _shape;
  Material _material;
};

class Static_body_storage {
  using Allocator = util::Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<util::Block, Static_body_storage>
  make(Allocator &allocator, util::Size max_static_bodies) {
    auto const block = allocator.alloc(memory_requirement(max_static_bodies));
    return {block, Static_body_storage{block, max_static_bodies}};
  }

  static constexpr util::Size
  memory_requirement(util::Size max_static_bodies) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_static_bodies),
        decltype(_available_handles)::memory_requirement(max_static_bodies),
        decltype(_occupancy_bits)::memory_requirement(max_static_bodies),
    });
  }

  constexpr Static_body_storage() = default;

  explicit Static_body_storage(util::Block block,
                               util::Size max_static_bodies) noexcept
      : Static_body_storage{block.begin, max_static_bodies} {}

  explicit Static_body_storage(std::byte *block_begin,
                               util::Size max_static_bodies) noexcept {
    auto allocator =
        Allocator{{block_begin, memory_requirement(max_static_bodies)}};
    _data = decltype(_data)::make(allocator, max_static_bodies).second;
    _data.resize(max_static_bodies);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_static_bodies).second;
    _available_handles.resize(max_static_bodies);
    for (auto i = util::Size{}; i != max_static_bodies; ++i) {
      _available_handles[i] =
          Static_body{static_cast<int>(max_static_bodies - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_static_bodies).second;
    _occupancy_bits.resize(max_static_bodies);
  }

  template <typename... Args> Static_body create(Args &&...args) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{"Out of space for static rigid bodies"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(std::forward<Args>(args)...);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Static_body static_body) {
    _available_handles.emplace_back(static_body);
    _occupancy_bits.reset(static_body.index());
  }

  Static_body_data const *data(Static_body static_body) const noexcept {
    return _data[static_body.index()].get();
  }

  Static_body_data *data(Static_body static_body) noexcept {
    return _data[static_body.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = util::Size{};
    for (auto i = util::Size{}; i != n && k != m; ++i) {
      if (_occupancy_bits[i]) {
        f(Static_body{i});
        ++k;
      }
    }
  }

private:
  util::List<util::Lifetime_box<Static_body_data>> _data;
  util::List<Static_body> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif