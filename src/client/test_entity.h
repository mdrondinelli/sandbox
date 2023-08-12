#ifndef MARLON_CLIENT_TEST_OBJECT_H
#define MARLON_CLIENT_TEST_OBJECT_H

#include <random>
#include <unordered_map>

#include "../graphics/graphics.h"
#include "../physics/physics.h"
#include "scene_diff_provider.h"

namespace marlon {
namespace client {
struct Test_entity_handle {
  std::uint64_t value;
};

struct Test_entity_create_info {};

class Test_entity_manager {
public:
  explicit Test_entity_manager(Scene_diff_provider const *scene_diff_provider,
                               graphics::Surface *surface,
                               physics::Space *space);

  ~Test_entity_manager();

  Test_entity_handle create_entity(Test_entity_create_info const &create_info);

  void destroy_entity(Test_entity_handle handle);

  void tick(float delta_time);

private:
  struct Entity : public physics::Particle_motion_callback {
    Test_entity_manager *manager{};
    graphics::Scene_node *scene_node{};
    graphics::Surface_instance *surface_instance{};
    physics::Particle_handle particle{};
    float time_alive{};

    void on_particle_motion(physics::Particle_motion_event const &event) final;
  };

  Scene_diff_provider const *_scene_diff_provider;
  graphics::Surface *_surface;
  physics::Space *_space;
  std::mt19937 _random_number_engine;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_reference_value;
};
} // namespace client
} // namespace marlon

#endif