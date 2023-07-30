#include "test_entity.h"

#include <array>

namespace marlon {
namespace client {
Test_entity_manager::Test_entity_manager(
    Scene_diff_provider const *scene_diff_provider, graphics::Surface *surface,
    physics::Space *space, Entity_construction_queue *entity_construction_queue,
    Entity_destruction_queue *entity_destruction_queue)
    : _scene_diff_provider{scene_diff_provider},
      _surface{surface}, _space{space},
      _entity_construction_queue{entity_construction_queue},
      _entity_destruction_queue{entity_destruction_queue},
      _random_number_engine{std::random_device{}()},
      _next_entity_reference_value{} {}

Test_entity_manager::~Test_entity_manager() {}

namespace {
template <typename Random_number_engine>
math::Vec3f sample_disk(Random_number_engine &random_number_engine) {
  std::uniform_real_distribution<float> distribution{0.0f, 1.0f};
  auto const u = distribution(random_number_engine);
  auto const v = distribution(random_number_engine);
  auto const angle = 2.0f * std::numbers::pi_v<float> * u;
  auto const radius = std::sqrt(v);
  return math::Vec3f{radius * std::cos(angle), 0.0f, -radius * std::sin(angle)};
}

template <typename Random_number_engine>
math::Vec3f sample_ball(Random_number_engine &random_number_engine) {
  std::uniform_real_distribution<float> distribution{-1.0f, 1.0f};
  math::Vec3f retval{0.0f, 0.0f, 0.0f};
  do {
    retval.x = distribution(random_number_engine);
    retval.y = distribution(random_number_engine);
    retval.z = distribution(random_number_engine);
  } while (math::length(retval) > 1.0f);
  return retval;
}
} // namespace

Entity_reference
Test_entity_manager::create_entity(Entity_create_info const &) {
  auto const collision_flags_array =
      std::array<std::uint64_t, 3>{0b01u, 0b01u, 0b01u};
  auto const collision_masks =
      std::array<std::uint64_t, 3>{0b01u, 0b01u, 0b11u};
  auto const centers = std::array<math::Vec3f, 3>{
      math::Vec3f{-3.0f, 0.0f, -3.0f}, math::Vec3f{0.0f, 3.5f, 0.0f},
      math::Vec3f{3.0f, 0.0f, 3.0f}};
  auto const radii = std::array<float, 3>{0.05f, 0.05f, 0.05f};
  auto const velocities = std::array<math::Vec3f, 3>{
      math::Vec3f{0.0f, 9.0f, 0.0f}, math::Vec3f{0.0f, -3.0f, 0.0f},
      math::Vec3f{0.0f, 9.0f, 0.0f}};
  auto const velocity_jitter_factors = std::array<float, 3>{0.0f, 0.0f, 0.0f};
  auto const accelerations = std::array<math::Vec3f, 3>{
      math::Vec3f{0.0f, -9.8f, 0.0f}, math::Vec3f{0.0f, -9.8f, 0.0f},
      math::Vec3f{0.0f, -9.8f, 0.0f}};
  auto const damping_factors = std::array<float, 3>{0.9f, 0.9f, 0.9f};
  auto const densities = std::array<float, 3>{1000.0f, 1000.0f, 1000.0f};
  std::uniform_int_distribution index_distribution{0, 5};
  auto const index = [](int n) {
    return n >= 3 ? 1 : n;
  }(index_distribution(_random_number_engine));
  auto const collision_flags = collision_flags_array[index];
  auto const collision_mask = collision_masks[index];
  auto const position =
      radii[index] * sample_ball(_random_number_engine) + centers[index];
  auto const velocity =
      velocity_jitter_factors[index] * sample_ball(_random_number_engine) +
      velocities[index];
  auto const acceleration = accelerations[index];
  auto const damping_factor = damping_factors[index];
  std::uniform_real_distribution<float> scale_distribution{0.01f, 0.03f};
  auto const density = densities[index];
  auto const scale = scale_distribution(_random_number_engine);
  Entity_reference const reference{_next_entity_reference_value};
  auto &value = _entities[reference];
  value.manager = this;
  value.reference = reference;
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  value.scene_node = scene_diff->record_scene_node_creation(
      {.translation = position, .scale = scale});
  value.surface_instance = scene_diff->record_surface_instance_creation(
      {.surface = _surface, .scene_node = value.scene_node});
  value.particle =
      _space->create_particle({.collision_flags = collision_flags,
                               .collision_mask = collision_mask,
                               .position = position,
                               .velocity = velocity,
                               .acceleration = acceleration,
                               .damping_factor = damping_factor,
                               .mass = density * scale * scale * scale,
                               .radius = scale,
                               .motion_callback = &value});
  ++_next_entity_reference_value;
  return reference;
}

void Test_entity_manager::destroy_entity(Entity_reference reference) {
  auto const it = _entities.find(reference);
  auto &value = it->second;
  auto const scene_diff = _scene_diff_provider->get_scene_diff();
  scene_diff->record_surface_instance_destruction(value.surface_instance);
  scene_diff->record_scene_node_destruction(value.scene_node);
  _space->destroy_particle(value.particle);
  _entities.erase(it);
}

void Test_entity_manager::tick_entities(float delta_time) {
  for (auto &[reference, value] : _entities) {
    value.time_alive += delta_time;
    if (value.time_alive > 2.0f) {
      _entity_destruction_queue->push(this, reference);
    }
  }
}

void Test_entity_manager::Test_entity::on_particle_motion(
    physics::Particle_motion_event const &event) {
  manager->_scene_diff_provider->get_scene_diff()
      ->record_scene_node_translation_continuous(scene_node, event.position);
}
} // namespace client
} // namespace marlon