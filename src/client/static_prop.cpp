#include "static_prop.h"

namespace marlon {
namespace client {
Static_prop_manager::Static_prop_manager(
    Static_prop_manager_create_info const &create_info)
    : _graphics{create_info.graphics}, _scene{create_info.scene},
      _surface_mesh{create_info.surface_mesh},
      _surface_material{create_info.surface_material},
      _surface_pretransform_3x4{create_info.surface_pretransform},
      _space{create_info.space}, _body_shape{create_info.body_shape},
      _body_material{create_info.body_material} {}

Static_prop_handle
Static_prop_manager::create(Static_prop_create_info const &create_info) {
  auto &value = _entities[_next_entity_handle_value];
  // TODO: consider exceptions in scene node creation
  auto const prop_transform = math::make_rigid_transform_mat4x4(
      create_info.position, create_info.orientation);
  auto const surface_pretransform_4x4 = math::Mat4x4f{
      _surface_pretransform_3x4[0], _surface_pretransform_3x4[1],
      _surface_pretransform_3x4[2], math::Vec4f{0.0f, 0.0f, 0.0f, 1.0f}};
  auto const surface_transform_4x4 = prop_transform * surface_pretransform_4x4;
  auto const surface_transform_3x4 =
      math::Mat3x4f{surface_transform_4x4[0], surface_transform_4x4[1],
                    surface_transform_4x4[2]};
  value.surface =
      _graphics->create_surface({.mesh = _surface_mesh,
                                 .material = _surface_material,
                                 .transform = surface_transform_3x4});
  _scene->add_surface(value.surface);
  value.body =
      _space->create_static_rigid_body({.collision_flags = 1,
                                        .collision_mask = 1,
                                        .position = create_info.position,
                                        .orientation = create_info.orientation,
                                        .shape = _body_shape,
                                        .material = _body_material});
  return {_next_entity_handle_value++};
}

void Static_prop_manager::destroy(Static_prop_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  _space->destroy_static_rigid_body(value.body);
  _scene->remove_surface(value.surface);
  _graphics->destroy_surface(value.surface);
  _entities.erase(it);
}
} // namespace client
} // namespace marlon