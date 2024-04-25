#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <functional>
#include <span>
#include <unordered_map>

#include "../math/vec.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "aabb_tree.h"
#include "material.h"
#include "object.h"

namespace marlon {
namespace physics {
class Particle_motion_callback;

struct Particle_data {
  Aabb_tree<Object_handle>::Node *aabb_tree_node{};
  Object_pair **neighbor_pairs{};
  Particle_motion_callback *motion_callback{};
  float radius{};
  float inverse_mass{};
  Material material{};
  math::Vec3f previous_position{};
  math::Vec3f position{};
  math::Vec3f velocity{};
  float waking_motion{};
  std::uint16_t neighbor_count{};
  bool marked{};
  bool awake{};
};

class World;

class Particle_motion_callback {
public:
  virtual ~Particle_motion_callback() = default;

  virtual void on_particle_motion(World const &world,
                                  Particle_handle particle) = 0;
};

class Particle_storage {
  using Allocator = util::Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<util::Block, Particle_storage>
  make(Allocator &allocator, std::size_t max_particles) {
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

  explicit Particle_storage(util::Block block,
                            std::size_t max_particles) noexcept
      : Particle_storage{block.begin, max_particles} {}

  explicit Particle_storage(void *block, std::size_t max_particles) noexcept {
    auto allocator =
        Allocator{util::make_block(block, memory_requirement(max_particles))};
    _data = decltype(_data)::make(allocator, max_particles).second;
    _data.resize(max_particles);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_particles).second;
    _available_handles.resize(max_particles);
    for (auto i = std::size_t{}; i != max_particles; ++i) {
      _available_handles[i] =
          Particle_handle{static_cast<Object_handle>(max_particles - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_particles).second;
    _occupancy_bits.resize(max_particles);
  }

  Particle_handle create(Particle_data const &data) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{"Capacity_error in Particle_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(data);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Particle_handle particle) {
    _available_handles.emplace_back(particle);
    _occupancy_bits.reset(particle.index());
  }

  Particle_data const *data(Particle_handle particle) const noexcept {
    return _data[particle.index()].get();
  }

  Particle_data *data(Particle_handle particle) noexcept {
    return _data[particle.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = std::size_t{};
    for (auto i = std::size_t{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        f(Particle_handle{static_cast<Object_handle>(i)});
        ++k;
      }
    }
  }

private:
  util::List<util::Lifetime_box<Particle_data>> _data;
  util::List<Particle_handle> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif