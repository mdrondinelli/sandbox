#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>
#include <thread>

#include "../util/size.h"
#include "particle.h"
#include "rigid_body.h"
#include "static_body.h"

namespace marlon {
namespace physics {
struct World_create_info {
  util::Size worker_thread_count{math::max(
      static_cast<util::Size>(std::thread::hardware_concurrency()) / 2 - 1,
      util::Size{0})};
  util::Size max_particles{10000};
  util::Size max_rigid_bodies{10000};
  util::Size max_static_bodies{100000};
  util::Size max_aabb_tree_leaf_nodes{100000};
  util::Size max_aabb_tree_internal_nodes{100000};
  util::Size max_neighbor_pairs{20000};
  util::Size max_neighbor_groups{10000};
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
  float delta_time{1.0f / 128.0f};
  int substep_count{10};
};

struct World_simulate_result {
  double total_wall_time;
  double broadphase_wall_time;
  double integration_wall_time;
  double narrowphase_wall_time;
  double position_solve_wall_time;
  double velocity_solve_wall_time;
};

class World {
public:
  World();

  explicit World(World_create_info const &create_info);

  ~World();

  Particle create_particle(Particle_create_info const &create_info);

  Rigid_body create_rigid_body(Rigid_body_create_info const &create_info);

  Static_body create_static_body(Static_body_create_info const &create_info);

  void destroy_object(Particle particle);

  void destroy_object(Rigid_body rigid_body);

  void destroy_object(Static_body handle);

  void destroy_object(Object handle);

  Particle_data const *data(Particle object) const noexcept;

  Particle_data *data(Particle object) noexcept;

  Rigid_body_data const *data(Rigid_body object) const noexcept;

  Rigid_body_data *data(Rigid_body object) noexcept;

  Static_body_data const *data(Static_body object) const noexcept;

  Static_body_data *data(Static_body object) noexcept;

  World_simulate_result simulate(World_simulate_info const &simulate_info);

private:
  class Impl;

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif
