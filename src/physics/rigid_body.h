#ifndef MARLON_PHYSICS_RIGID_BODY_H
#define MARLON_PHYSICS_RIGID_BODY_H

#include <bitset>

#include "../math/math.h"
#include "../util/bit_list.h"
#include "../util/lifetime_box.h"
#include "../util/list.h"
#include "broadphase.h"
#include "material.h"
#include "object.h"
#include "shape.h"

namespace marlon {
namespace physics {
class Rigid_body_motion_callback;

class Rigid_body_data {
  static auto constexpr asleep_flag_index = 0;
  static auto constexpr marked_flag_index = 1;

public:
  explicit Rigid_body_data(Broadphase_bvh::Node *bvh_node,
                           Rigid_body_motion_callback *motion_callback,
                           math::Vec3f const &position,
                           math::Vec3f const &velocity,
                           math::Quatf const &orientation,
                           math::Vec3f const &angular_velocity,
                           float motion,
                           float inverse_mass,
                           math::Mat3x3f const &inverse_inertia_tensor,
                           Shape const &shape,
                           Material const &material) noexcept
      : _bvh_node{bvh_node},
        _motion_callback{motion_callback},
        _position{position},
        _velocity{velocity},
        _orientation{orientation},
        _angular_velocity{angular_velocity},
        _motion{motion},
        _inverse_mass{inverse_mass},
        _inverse_inertia_tensor{inverse_inertia_tensor},
        _shape{shape},
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

  void push_neighbor(Object neighbor) noexcept {
    _neighbors[_neighbor_count++] = neighbor;
  }

  Rigid_body_motion_callback *motion_callback() const noexcept {
    return _motion_callback;
  }

  math::Vec3f const &position() const noexcept { return _position; }

  void position(math::Vec3f const &position) noexcept { _position = position; }

  math::Vec3f const &velocity() const noexcept { return _velocity; }

  void velocity(math::Vec3f const &velocity) { _velocity = velocity; }

  math::Quatf const &orientation() const noexcept { return _orientation; }

  void orientation(math::Quatf const &orientation) noexcept {
    _orientation = orientation;
  }

  math::Vec3f const &angular_velocity() const noexcept {
    return _angular_velocity;
  }

  void angular_velocity(math::Vec3f const &angular_velocity) noexcept {
    _angular_velocity = angular_velocity;
  }

  float motion() const noexcept { return _motion; }

  float mass() const noexcept { return 1.0f / _inverse_mass; }

  float inverse_mass() const noexcept { return _inverse_mass; }

  math::Mat3x3f const &inverse_inertia_tensor() const noexcept {
    return _inverse_inertia_tensor;
  }

  Shape const &shape() const noexcept { return _shape; }

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
    _angular_velocity = Vec3f::zero();
    _flags[asleep_flag_index] = true;
  }

  bool marked() const noexcept { return _flags[marked_flag_index]; }

  void marked(bool b) noexcept { _flags[marked_flag_index] = b; }

  void integrate(Object_integrate_info const &info) {
    using namespace math;
    _velocity += info.delta_velocity;
    _velocity *= info.damping_factor;
    _position += info.delta_time * _velocity;
    _angular_velocity *= info.damping_factor;
    _orientation +=
        Quatf{0.0f, 0.5f * info.delta_time * _angular_velocity} * _orientation;
    _orientation = normalize(_orientation);
    _motion = min(
        (1.0f - info.motion_smoothing_factor) * _motion +
            info.motion_smoothing_factor *
                (length_squared(_velocity) + length_squared(_angular_velocity)),
        info.motion_limit);
  }

private:
  Broadphase_bvh::Node *_bvh_node{};
  Object *_neighbors{};
  Rigid_body_motion_callback *_motion_callback{};
  math::Vec3f _position{};
  math::Vec3f _velocity{};
  math::Quatf _orientation{};
  math::Vec3f _angular_velocity{};
  float _motion;
  float _inverse_mass{};
  math::Mat3x3f _inverse_inertia_tensor{};
  Shape _shape;
  Material _material{};
  int _neighbor_count{};
  std::bitset<2> _flags;

  // bool visited() const noexcept { return flags[2]; }

  // void visited(bool b) noexcept { flags[2] = b; }
};

class World;

class Rigid_body_motion_callback {
public:
  virtual ~Rigid_body_motion_callback() = default;

  virtual void on_rigid_body_motion(World const &world,
                                    Rigid_body rigid_body) = 0;
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
      _available_handles[i] =
          Rigid_body{static_cast<int>(max_rigid_bodies - i - 1)};
    }
    _occupancy_bits = util::Bit_list::make(allocator, max_rigid_bodies).second;
    _occupancy_bits.resize(max_rigid_bodies);
  }

  template <typename... Args> Rigid_body create(Args &&...args) {
    if (_available_handles.empty()) {
      throw util::Capacity_error{
          "Capacity_error in Rigid_body_storage::create"};
    }
    auto const result = _available_handles.back();
    _available_handles.pop_back();
    _data[result.index()].construct(std::forward<Args>(args)...);
    _occupancy_bits.set(result.index());
    return result;
  }

  void destroy(Rigid_body rigid_body) {
    _available_handles.emplace_back(rigid_body);
    _occupancy_bits.reset(rigid_body.index());
  }

  Rigid_body_data const *data(Rigid_body rigid_body) const noexcept {
    return _data[rigid_body.index()].get();
  }

  Rigid_body_data *data(Rigid_body rigid_body) noexcept {
    return _data[rigid_body.index()].get();
  }

  template <typename F> void for_each(F &&f) {
    auto const n = _data.size();
    auto const m = _data.size() - _available_handles.size();
    auto k = util::Size{};
    for (auto i = util::Size{}; i != n && k != m; ++i) {
      if (_occupancy_bits.get(i)) {
        f(Rigid_body{static_cast<int>(i)});
        ++k;
      }
    }
  }

private:
  util::List<util::Lifetime_box<Rigid_body_data>> _data;
  util::List<Rigid_body> _available_handles;
  util::Bit_list _occupancy_bits;
};
} // namespace physics
} // namespace marlon

#endif