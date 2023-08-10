#ifndef MARLON_CLIENT_STATIC_PROP_ENTITY_H
#define MARLON_CLIENT_STATIC_PROP_ENTITY_H

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "entity.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
struct Static_prop_graphics_info {
  Scene_diff_provider const *scene_diff_provider{};
  graphics::Surface *surface{};
  float surface_scale{1.0f};
};

struct Static_prop_physics_info {
  physics::Space *space{};
  physics::Shape shape;
  physics::Material material;
};

struct Static_prop_entity_manager_create_info {
  Static_prop_graphics_info graphics;
  std::optional<Static_prop_physics_info> physics;
};

class Static_prop_entity_manager : public Entity_manager {
public:
  explicit Static_prop_entity_manager(
      Static_prop_entity_manager_create_info const &create_info);

  Entity_reference create_entity(Entity_create_info const &create_info) final;

  void destroy_entity(Entity_reference entity) final;

  void tick_entities(float delta_time) final;

private:
  struct Entity {
    graphics::Scene_node *scene_node{};
    graphics::Surface_instance *surface_instance{};
    std::optional<physics::Static_rigid_body_handle> static_rigid_body{};
  };

  Static_prop_graphics_info _graphics;
  std::optional<Static_prop_physics_info> _physics;
  std::uint64_t _next_entity_reference_value;
  std::unordered_map<Entity_reference, Entity> _entities;
};

struct Static_prop_entity_parameters {
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
};
} // namespace client
} // namespace marlon

#endif