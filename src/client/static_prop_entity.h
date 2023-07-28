#ifndef MARLON_CLIENT_STATIC_PROP_ENTITY_H
#define MARLON_CLIENT_STATIC_PROP_ENTITY_H

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "entity.h"

namespace marlon {
namespace client {
struct Static_prop_entity_manager_create_info {
  graphics::Graphics *graphics{};
  graphics::Scene_diff *const *scene_diff{};
  graphics::Surface *surface{};
  physics::Space *space{};
  physics::Shape *shape{};
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
    std::optional<physics::Static_rigid_body_reference> static_rigid_body{};
  };

  graphics::Graphics *_graphics;
  graphics::Scene_diff *const *_scene_diff;
  graphics::Surface *_surface;
  physics::Space *_space;
  physics::Shape *_shape;
  std::uint64_t _next_entity_reference_value;
  std::unordered_map<Entity_reference, Entity> _entities;
};

struct Static_prop_entity_parameters {
  math::Vec3f position;
  math::Quatf orientation;
};
} // namespace client
} // namespace marlon

#endif