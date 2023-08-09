#ifndef MARLON_CLIENT_STATIC_PROP_ENTITY_H
#define MARLON_CLIENT_STATIC_PROP_ENTITY_H

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "entity.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
struct Static_prop_entity_manager_create_info {
  Scene_diff_provider const *scene_diff_provider{};
  graphics::Surface *surface{};
  float surface_scale{1.0f};
  physics::Space *space{};
  physics::Shape shape;
  float static_friction_coefficient{0.0f};
  float dynamic_friction_coefficient{0.0f};
  float restitution_coefficient{0.0f};
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

  Scene_diff_provider const *_scene_diff_provider;
  graphics::Surface *_surface;
  float _surface_scale;
  physics::Space *_space;
  std::optional<physics::Shape> _shape;
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