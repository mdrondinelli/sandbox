#ifndef MARLON_PHYSICS_STATIC_BODY_H
#define MARLON_PHYSICS_STATIC_BODY_H

#include <cstdint>

#include "../math/math.h"
#include "material.h"
#include "object.h"
#include "shape.h"

namespace marlon {
namespace physics {
struct Static_body_data {
  Aabb_tree<Object_handle>::Node *aabb_tree_node;
  Shape shape;
  Material material;
  math::Vec3f position;
  math::Quatf orientation;
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

  explicit Static_body_storage(void *block,
                               util::Size max_static_bodies) noexcept {
    auto allocator = Allocator{
        util::make_block(block, memory_requirement(max_static_bodies))};
    _data = decltype(_data)::make(allocator, max_static_bodies).second;
    _data.resize(max_static_bodies);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_static_bodies).second;
    _available_handles.resize(max_static_bodies);
    for (auto i = util::Size{}; i != max_static_bodies; ++i) {
      _available_handles[i] = Static_body_handle{
          static_cast<Object_handle>(max_static_bodies - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_static_bodies).second;
    _occupancy_bits.resize(max_static_bodies);
  }

  Static_body_handle create(Static_body_data const &data) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{"Out of space for static rigid bodies"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(data);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Static_body_handle static_body) {
    _available_handles.emplace_back(static_body);
    _occupancy_bits.reset(static_body.index());
  }

  Static_body_data const *data(Static_body_handle static_body) const noexcept {
    return _data[static_body.index()].get();
  }

  Static_body_data *data(Static_body_handle static_body) noexcept {
    return _data[static_body.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _occupancy_bits.size();
    auto const m = _occupancy_bits.size() - _free_indices.size();
    auto k = util::Size{};
    for (auto i = util::Size{}; i != n; ++i) {
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
  util::List<util::Lifetime_box<Static_body_data>> _data;
  util::List<Static_body_handle> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif