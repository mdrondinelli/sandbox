#ifndef MARLON_CLIENT_TEST_OBJECT_H
#define MARLON_CLIENT_TEST_OBJECT_H

#include <random>
#include <unordered_map>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "entity.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
class Test_entity_manager : public Entity_manager {
public:
  explicit Test_entity_manager(
      Scene_diff_provider const *scene_diff_provider,
      graphics::Surface *surface, physics::Space *space,
      Entity_destruction_queue *entity_destruction_queue);

  ~Test_entity_manager();

  Entity_reference create_entity(Entity_create_info const &create_info) final;

  void destroy_entity(Entity_reference entity) final;

  void tick_entities(float delta_time) final;

private:
  struct Test_entity : public physics::Particle_motion_callback {
    Test_entity_manager *manager{};
    Entity_reference reference{};
    graphics::Scene_node *scene_node{};
    graphics::Surface_instance *surface_instance{};
    physics::Particle_handle particle{};
    float time_alive{};

    void on_particle_motion(physics::Particle_motion_event const &event) final;
  };

  Scene_diff_provider const *_scene_diff_provider;
  graphics::Surface *_surface;
  physics::Space *_space;
  Entity_destruction_queue *_entity_destruction_queue;
  std::mt19937 _random_number_engine;
  std::uint64_t _next_entity_reference_value;
  std::unordered_map<Entity_reference, Test_entity> _entities;
};
} // namespace client
} // namespace marlon

#endif