#include "entity.h"

namespace marlon {
namespace client {
void Entity_construction_queue::push(Entity_manager *manager,
                                     Entity_create_info const &create_info) {
  _elements.emplace_back(manager, create_info);
}

void Entity_construction_queue::consume() {
  for (auto &[manager, create_info] : _elements) {
    manager->create_entity(create_info);
  }
  _elements.clear();
}

void Entity_destruction_queue::push(Entity_manager *manager,
                                    Entity_reference entity) {
  _elements.emplace_back(manager, entity);
}

void Entity_destruction_queue::consume() {
  for (auto [manager, entity] : _elements) {
    manager->destroy_entity(entity);
  }
  _elements.clear();
}
} // namespace client
} // namespace marlon