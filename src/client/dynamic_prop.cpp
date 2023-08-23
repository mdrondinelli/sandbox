#include "dynamic_prop.h"

#include <iostream>

namespace marlon {
namespace client {
Dynamic_prop_manager::Dynamic_prop_manager(
    Dynamic_prop_manager_create_info const &create_info)
    : _graphics{create_info.graphics}, _scene{create_info.scene},
      _surface_mesh{create_info.surface_mesh},
      _surface_material{create_info.surface_material},
      _surface_pretransform_3x4{create_info.surface_pretransform},
      _space{create_info.space}, _body_mass{create_info.body_mass},
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
  auto const surface_transform_3x4 =
      math::Mat3x4f{surface_transform_4x4[0], surface_transform_4x4[1],
                    surface_transform_4x4[2]};
  value.surface =
      _graphics->create_surface({.mesh = _surface_mesh,
                                 .material = _surface_material,
                                 .transform = surface_transform_3x4});
  try {
    value.body = _space->create_dynamic_rigid_body(
        {.motion_callback = &value,
         .collision_flags = 1,
         .collision_mask = 1,
         .position = create_info.position,
         .velocity = create_info.velocity,
         .orientation = create_info.orientation,
         .angular_velocity = create_info.angular_velocity,
         .mass = _body_mass,
         .inertia_tensor = _body_inertia_tensor,
         .shape = _body_shape,
         .material = _body_material});
  } catch (...) {
    _graphics->destroy_surface(value.surface);
    throw;
  }
  try {
    _scene->add_surface(value.surface);
  } catch (...) {
    _space->destroy_dynamic_rigid_body(value.body);
    _graphics->destroy_surface(value.surface);
    throw;
  }
  return {_next_entity_handle_value++};
}

void Dynamic_prop_manager::destroy(Dynamic_prop_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  _space->destroy_dynamic_rigid_body(value.body);
  _scene->remove_surface(value.surface);
  _graphics->destroy_surface(value.surface);
  _entities.erase(it);
}

void Dynamic_prop_manager::Entity::on_dynamic_rigid_body_motion(
    physics::Dynamic_rigid_body_motion_event const &event) {
  auto const prop_transform =
      math::Mat4x4f::rigid(event.position, event.orientation);
  auto const &surface_pretransform_3x4 = manager->_surface_pretransform_3x4;
  auto const surface_pretransform_4x4 = math::Mat4x4f{
      surface_pretransform_3x4, {0.0f, 0.0f, 0.0f, 1.0}};
  auto const surface_transform_4x4 = prop_transform * surface_pretransform_4x4;
  auto const surface_transform_3x4 =
      math::Mat3x4f{surface_transform_4x4[0], surface_transform_4x4[1],
                    surface_transform_4x4[2]};
  surface->set_transform(surface_transform_3x4);
}
} // namespace client
} // namespace marlon