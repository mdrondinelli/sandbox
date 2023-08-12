#include "static_prop.h"

namespace marlon {
namespace client {
Static_prop_manager::Static_prop_manager(
    Static_prop_manager_create_info const &create_info)
    : _scene_diff_provider{create_info.scene_diff_provider},
      _surface{create_info.surface}, _surface_scale{create_info.surface_scale},
      _space{create_info.space}, _shape{create_info.shape},
      _material{create_info.material} {}

Static_prop_handle
Static_prop_manager::create(Static_prop_create_info const &create_info) {
  auto &value = _entities[_next_entity_handle_value];
  // TODO: consider exceptions in scene node creation
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  value.scene_node = scene_diff->record_scene_node_creation(
      {.translation = create_info.position,
       .rotation = create_info.orientation,
       .scale = _surface_scale});
  value.surface_instance = scene_diff->record_surface_instance_creation(
      {.surface = _surface, .scene_node = value.scene_node});
  value.static_rigid_body =
      _space->create_static_rigid_body({.collision_flags = 1,
                                        .collision_mask = 1,
                                        .position = create_info.position,
                                        .orientation = create_info.orientation,
                                        .shape = _shape,
                                        .material = _material});
  return {_next_entity_handle_value++};
}

void Static_prop_manager::destroy(Static_prop_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  scene_diff->record_surface_instance_destruction(value.surface_instance);
  scene_diff->record_scene_node_destruction(value.scene_node);
  _space->destroy_static_rigid_body(value.static_rigid_body);
  _entities.erase(it);
}
} // namespace client
} // namespace marlon