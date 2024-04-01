#ifndef MARLON_CLIENT_TEST_OBJECT_H
#define MARLON_CLIENT_TEST_OBJECT_H

#include <random>
#include <unordered_map>

#include "../graphics/graphics.h"
#include "../physics/physics.h"

namespace marlon {
namespace client {
struct Test_entity_handle {
  std::uint64_t value;
};

struct Test_entity_manager_create_info {
  graphics::Graphics *graphics;
  graphics::Scene *scene;
  graphics::Mesh *surface_mesh;
  graphics::Material *surface_material;
  physics::World *space;
};

struct Test_entity_create_info {};

class Test_entity_manager {
public:
  explicit Test_entity_manager(
      Test_entity_manager_create_info const &create_info);

  ~Test_entity_manager();

  Test_entity_handle create_entity(Test_entity_create_info const &create_info);

  void destroy_entity(Test_entity_handle handle);

  void tick(float delta_time);

private:
  struct Entity : public physics::Particle_motion_callback {
    Test_entity_manager *manager{};
    graphics::Surface *surface{};
    physics::Particle_handle particle{};
    float time_alive{};

    void on_particle_motion(physics::Particle_motion_event const &event) final;
  };

  graphics::Graphics *_graphics;
  graphics::Scene *_scene;
  graphics::Mesh *_surface_mesh;
  graphics::Material *_surface_material;
  physics::World *_space;
  std::mt19937 _random_number_engine;
  std::unordered_map<std::uint64_t, Entity> _entities;
  std::uint64_t _next_entity_reference_value;
};
} // namespace client
} // namespace marlon

#endif