#ifndef MARLON_CLIENT_DYNAMIC_PROP_H
#define MARLON_CLIENT_DYNAMIC_PROP_H

#include <cstdint>
#include <unordered_map>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
struct Dynamic_prop_handle {
  std::uint64_t value;
};

struct Dynamic_prop_manager_create_info {
  Scene_diff_provider const *scene_diff_provider;
  graphics::Surface surface;
  float surface_scale;
  physics::Space *space;
  float mass;
  math::Mat3x3f inertia_tensor;
  physics::Shape shape;
  physics::Material material;
};

struct Dynamic_prop_create_info {
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
  math::Vec3f angular_velocity{math::Vec3f::zero()};
};

class Dynamic_prop_manager {
public:
  explicit Dynamic_prop_manager(
      Dynamic_prop_manager_create_info const &create_info);

  Dynamic_prop_manager(Dynamic_prop_manager const &other) = delete;

  Dynamic_prop_manager &operator=(Dynamic_prop_manager const &other) = delete;

  Dynamic_prop_handle create(Dynamic_prop_create_info const &create_info);

  void destroy(Dynamic_prop_handle handle);

private:
  struct Entity : public physics::Dynamic_rigid_body_motion_callback {
    Dynamic_prop_manager *manager;
    graphics::Scene_node *scene_node;
    graphics::Surface_instance *surface_instance;
    physics::Dynamic_rigid_body_handle rigid_body;

    void on_dynamic_rigid_body_motion(
        physics::Dynamic_rigid_body_motion_event const &event) final;
  };

  Scene_diff_provider const *_scene_diff_provider;
  graphics::Surface _surface;
  float _surface_scale;
  physics::Space *_space;
  float _mass;
  math::Mat3x3f _inertia_tensor;
  physics::Shape _shape;
  physics::Material _material;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_handle_value{};
};
} // namespace client
} // namespace marlon

#endif