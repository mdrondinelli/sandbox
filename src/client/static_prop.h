#ifndef MARLON_CLIENT_STATIC_PROP_ENTITY_H
#define MARLON_CLIENT_STATIC_PROP_ENTITY_H

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
struct Static_prop_handle {
  std::uint64_t value;
};

struct Static_prop_manager_create_info {
  Scene_diff_provider const *scene_diff_provider{};
  graphics::Surface *surface{};
  float surface_scale{1.0f};
  physics::Space *space;
  physics::Shape shape;
  physics::Material material;
};

struct Static_prop_create_info {
  math::Vec3f position{math::Vec3f::zero()};
  math::Quatf orientation{math::Quatf::identity()};
};

class Static_prop_manager {
public:
  explicit Static_prop_manager(
      Static_prop_manager_create_info const &create_info);

  Static_prop_handle create(Static_prop_create_info const &create_info);

  void destroy(Static_prop_handle handle);

private:
  struct Entity {
    graphics::Scene_node *scene_node{};
    graphics::Surface_instance *surface_instance{};
    physics::Static_rigid_body_handle static_rigid_body{};
  };

  Scene_diff_provider const *_scene_diff_provider;
  graphics::Surface *_surface;
  float _surface_scale;
  physics::Space *_space;
  physics::Shape _shape;
  physics::Material _material;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_handle_value{};
};
} // namespace client
} // namespace marlon

#endif