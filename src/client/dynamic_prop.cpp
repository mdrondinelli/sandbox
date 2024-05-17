#include "dynamic_prop.h"

#include <iostream>

namespace marlon {
namespace client {
Dynamic_prop_manager::Dynamic_prop_manager(
    Dynamic_prop_manager_create_info const &create_info)
    : _scene{create_info.scene},
      _surface_mesh{create_info.surface_mesh},
      _surface_material{create_info.surface_material},
      _surface_pretransform_3x4{create_info.surface_pretransform},
      _space{create_info.space},
      _body_mass{create_info.body_mass},
      _body_inertia_tensor{create_info.body_inertia_tensor},
      _body_shape{create_info.body_shape},
      _body_material{create_info.body_material} {}

Dynamic_prop_handle
Dynamic_prop_manager::create(Dynamic_prop_create_info const &create_info) {
  auto &value = _entities[_next_entity_handle_value];
  value.manager = this;
  auto const prop_transform =
      math::Mat4x4f::rigid(create_info.position, create_info.orientation);
  auto const surface_pretransform_4x4 =
      math::Mat4x4f{_surface_pretransform_3x4, {0.0f, 0.0f, 0.0f, 1.0}};
  auto const surface_transform_4x4 = prop_transform * surface_pretransform_4x4;
  auto const surface_transform_3x4 = math::Mat3x4f{surface_transform_4x4[0],
                                                   surface_transform_4x4[1],
                                                   surface_transform_4x4[2]};
  value.surface = {
      .mesh = _surface_mesh,
      .material = _surface_material,
      .transform = surface_transform_3x4,
  };
  _scene->add(&value.surface);
  try {
    value.body = _space->create_rigid_body({
        .motion_callback = &value,
        .shape = _body_shape,
        .mass = _body_mass,
        .inertia_tensor = _body_inertia_tensor,
        .material = _body_material,
        .position = create_info.position,
        .velocity = create_info.velocity,
        .orientation = create_info.orientation,
        .angular_velocity = create_info.angular_velocity,
    });
  } catch (...) {
    _scene->remove(&value.surface);
    throw;
  }
  return {_next_entity_handle_value++};
}

void Dynamic_prop_manager::destroy(Dynamic_prop_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  _space->destroy_rigid_body(value.body);
  _scene->remove(&value.surface);
  _entities.erase(it);
}

physics::Rigid_body
Dynamic_prop_manager::get_rigid_body(Dynamic_prop_handle prop) const noexcept {
  return _entities.at(prop.value).body;
}

void Dynamic_prop_manager::Entity::on_rigid_body_motion(
    physics::World const &world, physics::Rigid_body rigid_body) {
  auto const position = world.data(rigid_body)->position();
  auto const orientation = world.data(rigid_body)->orientation();
  auto const prop_transform = math::Mat4x4f::rigid(position, orientation);
  auto const &surface_pretransform_3x4 = manager->_surface_pretransform_3x4;
  auto const surface_pretransform_4x4 =
      math::Mat4x4f{surface_pretransform_3x4, {0.0f, 0.0f, 0.0f, 1.0}};
  auto const surface_transform_4x4 = prop_transform * surface_pretransform_4x4;
  auto const surface_transform_3x4 = math::Mat3x4f{surface_transform_4x4[0],
                                                   surface_transform_4x4[1],
                                                   surface_transform_4x4[2]};
  surface.transform = surface_transform_3x4;
}
} // namespace client
} // namespace marlon