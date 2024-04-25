#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>

#include "../util/thread_pool.h"
#include "particle.h"
#include "rigid_body.h"
#include "static_body.h"

namespace marlon {
namespace physics {
struct World_create_info {
  std::size_t max_aabb_tree_leaf_nodes{100000};
  std::size_t max_aabb_tree_internal_nodes{100000};
  std::size_t max_particles{10000};
  std::size_t max_rigid_bodies{10000};
  std::size_t max_static_bodies{100000};
  std::size_t max_neighbor_pairs{20000};
  std::size_t max_neighbor_groups{10000};
  math::Vec3f gravitational_acceleration{math::Vec3f::zero()};
};

struct Particle_create_info {
  Particle_motion_callback *motion_callback{};
  float radius{0.0f};
  float mass{1.0f};
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
};

struct Rigid_body_create_info {
  Rigid_body_motion_callback *motion_callback{};
  Shape shape;
  float mass{1.0f};
  math::Mat3x3f inertia_tensor{math::Mat3x3f::identity()};
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec3f angular_velocity{math::Vec3f::zero()};
};

struct Static_body_create_info {
  Shape shape;
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
};

struct World_simulate_info {
  util::Thread_pool *thread_pool;
  float delta_time{1.0f / 64.0f};
  int substep_count{16};
};

class World {
public:
  explicit World(World_create_info const &create_info);

  ~World();

  Particle_handle create_particle(Particle_create_info const &create_info);

  void destroy_particle(Particle_handle particle);

  bool is_awake(Particle_handle particle) const noexcept;

  float get_waking_motion(Particle_handle particle) const noexcept;

  math::Vec3f get_position(Particle_handle particle) const noexcept;

  Rigid_body_handle
  create_rigid_body(Rigid_body_create_info const &create_info);

  void destroy_rigid_body(Rigid_body_handle rigid_body);

  bool is_awake(Rigid_body_handle rigid_body) const noexcept;

  float get_waking_motion(Rigid_body_handle rigid_body) const noexcept;

  math::Vec3f get_position(Rigid_body_handle rigid_body) const noexcept;

  math::Quatf get_orientation(Rigid_body_handle rigid_body) const noexcept;

  Static_body_handle
  create_static_body(Static_body_create_info const &create_info);

  void destroy_rigid_body(Static_body_handle handle);

  void simulate(World_simulate_info const &simulate_info);

private:
  class Impl;

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif