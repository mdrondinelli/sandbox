#ifndef MARLON_PHYSICS_SPACE_H
#define MARLON_PHYSICS_SPACE_H

#include <memory>

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
  std::size_t max_particle_particle_neighbor_pairs{10000};
  std::size_t max_particle_rigid_body_neighbor_pairs{10000};
  std::size_t max_particle_static_body_neighbor_pairs{10000};
  std::size_t max_rigid_body_rigid_body_neighbor_pairs{10000};
  std::size_t max_rigid_body_static_body_neighbor_pairs{10000};
  std::size_t max_neighbor_groups{10000};
  std::size_t max_contact_group_fringe_size{1000};
  std::size_t max_contact_group_size{10000};
  math::Vec3f gravitational_acceleration{math::Vec3f::zero()};
};

struct World_simulate_info {
  float delta_time{1.0f / 64.0f};
  int substep_count{16};
  int min_desired_position_iterations_per_contact{1};
  int max_desired_position_iterations_per_contact{4};
  int max_position_iterations_per_contact_group{256};
  int min_desired_velocity_iterations_per_contact{1};
  int max_desired_velocity_iterations_per_contact{4};
  int max_velocity_iterations_per_contact_group{256};
  float early_out_contact_separation{-1.0f / 1024.0f};
  float early_out_contact_separating_velocity{-1.0f / 128.0f};
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

  void destroy_static_body(Static_body_handle handle);

  void simulate(World_simulate_info const &simulate_info);

private:
  class Impl;

  std::unique_ptr<Impl> _impl;
};
} // namespace physics
} // namespace marlon

#endif