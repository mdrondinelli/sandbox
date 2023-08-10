#include "static_prop_entity.h"

namespace marlon {
namespace client {
Static_prop_entity_manager::Static_prop_entity_manager(
    Static_prop_entity_manager_create_info const &create_info)
    : _graphics{create_info.graphics}, _physics{create_info.physics},
      _next_entity_reference_value{} {}

Entity_reference Static_prop_entity_manager::create_entity(
    Entity_create_info const &create_info) {
  auto parameters = Static_prop_entity_parameters{};
  std::memcpy(&parameters, create_info.parameters, sizeof(parameters));
  auto const reference = Entity_reference{_next_entity_reference_value};
  auto &value = _entities[reference];
  // TODO: consider exceptions in scene node creation
  auto const scene_diff = _graphics.scene_diff_provider->get_scene_diff();
  value.scene_node = scene_diff->record_scene_node_creation(
      {.translation = parameters.position,
       .rotation = parameters.orientation,
       .scale = _graphics.surface_scale});
  value.surface_instance = scene_diff->record_surface_instance_creation(
      {.surface = _graphics.surface, .scene_node = value.scene_node});
  if (_physics) {
    value.static_rigid_body = _physics->space->create_static_rigid_body(
        {.collision_flags = 1,
         .collision_mask = 1,
         .position = parameters.position,
         .orientation = parameters.orientation,
         .shape = _physics->shape,
         .material = _physics->material});
  }
  return reference;
}

void Static_prop_entity_manager::destroy_entity(Entity_reference reference) {
  auto const it = _entities.find(reference);
  auto &value = it->second;
  auto const scene_diff = _graphics.scene_diff_provider->get_scene_diff();
  scene_diff->record_surface_instance_destruction(value.surface_instance);
  scene_diff->record_scene_node_destruction(value.scene_node);
  if (_physics) {
    _physics->space->destroy_static_rigid_body(*value.static_rigid_body);
  }
  _entities.erase(it);
}

void Static_prop_entity_manager::tick_entities(float /*delta_time*/) {}
} // namespace client
} // namespace marlon