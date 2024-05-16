#ifndef MARLON_CLIENT_DYNAMIC_PROP_H
#define MARLON_CLIENT_DYNAMIC_PROP_H

#include <cstdint>
#include <unordered_map>

#include "../graphics/graphics.h"
#include "../physics/physics.h"

namespace marlon {
namespace client {
struct Dynamic_prop_handle {
  std::uint64_t value;
};

struct Dynamic_prop_manager_create_info {
  graphics::Scene *scene{};
  graphics::Surface_mesh *surface_mesh{};
  graphics::Surface_material surface_material{};
  math::Mat3x4f surface_pretransform{math::Mat3x4f::identity()};
  physics::World *space;
  float body_mass{1.0f};
  math::Mat3x3f body_inertia_tensor{math::Mat3x3f::identity()};
  physics::Shape body_shape;
  physics::Material body_material;
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

  physics::Rigid_body get_rigid_body(Dynamic_prop_handle prop) const noexcept;

private:
  struct Entity : public physics::Rigid_body_motion_callback {
    Dynamic_prop_manager *manager;
    graphics::Surface *surface;
    physics::Rigid_body body;

    void on_rigid_body_motion(physics::World const &world,
                              physics::Rigid_body rigid_body) final;
  };

  graphics::Scene *_scene;
  graphics::Surface_mesh *_surface_mesh;
  graphics::Surface_material _surface_material;
  math::Mat3x4f _surface_pretransform_3x4;
  physics::World *_space;
  float _body_mass;
  math::Mat3x3f _body_inertia_tensor;
  physics::Shape _body_shape;
  physics::Material _body_material;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_handle_value{};
};
} // namespace client
} // namespace marlon

#endif