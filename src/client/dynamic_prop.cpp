#include "dynamic_prop.h"

#include <iostream>

namespace marlon {
namespace client {
Dynamic_prop_manager::Dynamic_prop_manager(
    Dynamic_prop_manager_create_info const &create_info)
    : _scene_diff_provider{create_info.scene_diff_provider},
      _surface{create_info.surface}, _surface_scale{create_info.surface_scale},
      _space{create_info.space}, _mass{create_info.mass},
      _inertia_tensor{create_info.inertia_tensor}, _shape{create_info.shape},
      _material{create_info.material} {}

Dynamic_prop_handle
Dynamic_prop_manager::create(Dynamic_prop_create_info const &create_info) {
  auto &value = _entities[_next_entity_handle_value];
  value.manager = this;
  // TODO: consider exceptions in scene node creation
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  value.scene_node = scene_diff->record_scene_node_creation(
      {.translation = create_info.position,
       .rotation = create_info.orientation,
       .scale = _surface_scale});
  value.surface_instance = scene_diff->record_surface_instance_creation(
      {.surface = _surface, .scene_node = value.scene_node});
  value.rigid_body = _space->create_dynamic_rigid_body(
      {.motion_callback = &value,
       .collision_flags = 1,
       .collision_mask = 1,
       .position = create_info.position,
       .velocity = create_info.velocity,
       .orientation = create_info.orientation,
       .angular_velocity = create_info.angular_velocity,
       .mass = _mass,
       .inertia_tensor = _inertia_tensor,
       .shape = _shape,
       .material = _material});
  return {_next_entity_handle_value++};
}

void Dynamic_prop_manager::destroy(Dynamic_prop_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  scene_diff->record_surface_instance_destruction(value.surface_instance);
  scene_diff->record_scene_node_destruction(value.scene_node);
  _space->destroy_dynamic_rigid_body(value.rigid_body);
  _entities.erase(it);
}

void Dynamic_prop_manager::Entity::on_dynamic_rigid_body_motion(
    physics::Dynamic_rigid_body_motion_event const &event) {
  auto const scene_diff = manager->_scene_diff_provider->get_scene_diff();
  scene_diff->record_scene_node_translation_continuous(scene_node,
                                                       event.position);
  scene_diff->record_scene_node_rotation_continuous(scene_node,
                                                    event.orientation);
  // std::cout << "rigid body motion\n";
}
} // namespace client
} // namespace marlon