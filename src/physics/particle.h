#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <bitset>
#include <functional>
#include <span>

#include "../math/vec.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "broadphase.h"
#include "material.h"
#include "object.h"

namespace marlon {
namespace physics {
class Particle_motion_callback;

class Particle_data {
  static auto constexpr asleep_flag_index = 0;
  static auto constexpr marked_flag_index = 1;

public:
  explicit Particle_data(Broadphase_bvh::Node *bvh_node,
                         Particle_motion_callback *motion_callback,
                         math::Vec3f const &position,
                         math::Vec3f const &velocity,
                         float motion,
                         float inverse_mass,
                         float radius,
                         Material const &material) noexcept
      : _bvh_node{bvh_node},
        _motion_callback{motion_callback},
        _position{position},
        _velocity{velocity},
        _motion{motion},
        _inverse_mass{inverse_mass},
        _radius{radius},
        _material{material} {}

  Broadphase_bvh::Node const *bvh_node() const noexcept { return _bvh_node; }

  Broadphase_bvh::Node *bvh_node() noexcept { return _bvh_node; }

  std::span<Object const> neighbors() const noexcept {
    return {_neighbors, static_cast<std::size_t>(_neighbor_count)};
  }

  std::span<Object> neighbors() noexcept {
    return {_neighbors, static_cast<std::size_t>(_neighbor_count)};
  }

  void reset_neighbors() noexcept {
    _neighbors = nullptr;
    _neighbor_count = 0;
  }

  void count_neighbor() noexcept { ++_neighbor_count; }

  void reserve_neighbors(util::List<Object> &neighbors) {
    _neighbors = neighbors.data() + neighbors.size();
    neighbors.resize(neighbors.size() + _neighbor_count);
    _neighbor_count = 0;
  }

  void push_neighbor(Object neighbor) {
    _neighbors[_neighbor_count++] = neighbor;
  }

  Particle_motion_callback *motion_callback() const noexcept {
    return _motion_callback;
  }

  math::Vec3f const &position() const noexcept { return _position; }

  void position(math::Vec3f const &position) noexcept { _position = position; }

  math::Vec3f const &velocity() const noexcept { return _velocity; }

  void velocity(math::Vec3f const &velocity) noexcept { _velocity = velocity; }

  float motion() const noexcept { return _motion; }

  float mass() const noexcept { return 1.0f / _inverse_mass; }

  float inverse_mass() const noexcept { return _inverse_mass; }

  float radius() const noexcept { return _radius; }

  Material const &material() const noexcept { return _material; }

  bool awake() const noexcept { return !asleep(); }

  bool asleep() const noexcept { return _flags[asleep_flag_index]; }

  void wake(float motion) noexcept {
    _motion = motion;
    _flags[asleep_flag_index] = false;
  }

  void sleep() noexcept {
    using namespace math;
    _velocity = Vec3f::zero();
    _flags[asleep_flag_index] = true;
  }

  bool marked() const noexcept { return _flags[marked_flag_index]; }

  void marked(bool b) noexcept { _flags[marked_flag_index] = b; }

  void integrate(Object_integrate_info const &info) noexcept {
    using namespace math;
    _velocity += info.delta_velocity;
    _velocity *= info.damping_factor;
    _position += info.delta_time * _velocity;
    _motion = min((1.0f - info.motion_smoothing_factor) * _motion +
                      info.motion_smoothing_factor * length_squared(_velocity),
                  info.motion_limit);
  }

private:
  Broadphase_bvh::Node *_bvh_node{};
  Object *_neighbors{};
  Particle_motion_callback *_motion_callback{};
  math::Vec3f _position{};
  math::Vec3f _velocity{};
  float _motion{};
  float _inverse_mass{};
  float _radius{};
  Material _material{};
  int _neighbor_count{};
  std::bitset<2> _flags;
};

class World;

class Particle_motion_callback {
public:
  virtual ~Particle_motion_callback() = default;

  virtual void on_particle_motion(World const &world, Particle particle) = 0;
};

class Particle_storage {
  using Allocator = util::Stack_allocator<>;

public:
  template <typename Allocator>
  static std::pair<util::Block, Particle_storage>
  make(Allocator &allocator, util::Size max_particles) {
    auto const block = allocator.alloc(memory_requirement(max_particles));
    return {block, Particle_storage{block, max_particles}};
  }

  static constexpr util::Size
  memory_requirement(util::Size max_particles) noexcept {
    return Allocator::memory_requirement({
        decltype(_data)::memory_requirement(max_particles),
        decltype(_available_handles)::memory_requirement(max_particles),
        decltype(_occupancy_bits)::memory_requirement(max_particles),
    });
  }

  constexpr Particle_storage() noexcept = default;

  explicit Particle_storage(util::Block block,
                            util::Size max_particles) noexcept
      : Particle_storage{block.begin, max_particles} {}

  explicit Particle_storage(void *block, util::Size max_particles) noexcept {
    auto allocator =
        Allocator{util::make_block(block, memory_requirement(max_particles))};
    _data = decltype(_data)::make(allocator, max_particles).second;
    _data.resize(max_particles);
    _available_handles =
        decltype(_available_handles)::make(allocator, max_particles).second;
    _available_handles.resize(max_particles);
    for (auto i = util::Size{}; i != max_particles; ++i) {
      _available_handles[i] = Particle{static_cast<int>(max_particles - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_particles).second;
    _occupancy_bits.resize(max_particles);
  }

  template <typename... Args> Particle create(Args &&...args) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{"Capacity_error in Particle_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(std::forward<Args>(args)...);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Particle particle) {
    _available_handles.emplace_back(particle);
    _occupancy_bits.reset(particle.index());
  }

  Particle_data const *data(Particle particle) const noexcept {
    return _data[particle.index()].get();
  }

  Particle_data *data(Particle particle) noexcept {
    return _data[particle.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = util::Size{};
    for (auto i = util::Size{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        f(Particle{static_cast<int>(i)});
        ++k;
      }
    }
  }

private:
  util::List<util::Lifetime_box<Particle_data>> _data;
  util::List<Particle> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif