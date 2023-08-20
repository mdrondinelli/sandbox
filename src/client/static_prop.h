#ifndef MARLON_CLIENT_STATIC_PROP_ENTITY_H
#define MARLON_CLIENT_STATIC_PROP_ENTITY_H

#include "../graphics/graphics.h"
#include "../physics/physics.h"

namespace marlon {
namespace client {
struct Static_prop_handle {
  std::uint64_t value;
};

struct Static_prop_manager_create_info {
  graphics::Graphics *graphics{};
  graphics::Scene *scene{};
  graphics::Mesh *surface_mesh{};
  graphics::Material *surface_material{};
  math::Mat3x4f surface_pretransform{math::Mat3x4f::identity()};
  physics::Space *space;
  physics::Shape body_shape;
  physics::Material body_material;
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
    graphics::Surface *surface{};
    physics::Static_rigid_body_handle body{};
  };

  graphics::Graphics *_graphics;
  graphics::Scene *_scene;
  graphics::Mesh *_surface_mesh;
  graphics::Material *_surface_material;
  math::Mat3x4f _surface_pretransform_3x4;
  physics::Space *_space;
  physics::Shape _body_shape;
  physics::Material _body_material;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_handle_value{};
};
} // namespace client
} // namespace marlon

#endif