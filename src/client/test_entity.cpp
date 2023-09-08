#include "test_entity.h"

#include <array>

namespace marlon {
namespace client {
Test_entity_manager::Test_entity_manager(
    Test_entity_manager_create_info const &create_info)
    : _graphics{create_info.graphics}, _scene{create_info.scene},
      _surface_mesh{create_info.surface_mesh},
      _surface_material{create_info.surface_material},
      _space{create_info.space}, _random_number_engine{std::random_device{}()},
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

Test_entity_handle
Test_entity_manager::create_entity(Test_entity_create_info const &) {
  auto const collision_flags_array =
      std::array<std::uint64_t, 3>{0b01u, 0b01u, 0b01u};
  auto const collision_masks =
      std::array<std::uint64_t, 3>{0b01u, 0b01u, 0b11u};
  auto const centers = std::array<math::Vec3f, 3>{
      math::Vec3f{-3.0f, 0.5f, -3.0f}, math::Vec3f{0.0f, 5.5f, 0.0f},
      math::Vec3f{3.0f, 0.5f, 3.0f}};
  auto const radii = std::array<float, 3>{0.1f, 0.5f, 0.1f};
  auto const velocities = std::array<math::Vec3f, 3>{
      math::Vec3f{0.0f, 9.0f, 0.0f}, math::Vec3f{0.0f, 0.0f, 0.0f},
      math::Vec3f{0.0f, 9.0f, 0.0f}};
  auto const velocity_jitter_factors = std::array<float, 3>{0.0f, 0.0f, 0.0f};
  auto const densities = std::array<float, 3>{500.0f, 500.0f, 500.0f};
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
  std::uniform_real_distribution<float> scale_distribution{0.02f, 0.06f};
  auto const density = densities[index];
  auto const scale = scale_distribution(_random_number_engine);
  auto &value = _entities[_next_entity_reference_value];
  value.manager = this;
  value.surface = _graphics->create_surface(
      {.mesh = _surface_mesh,
       .material = _surface_material,
       .transform = math::Mat3x4f{{scale, 0.0f, 0.0f, position.x},
                                  {0.0f, scale, 0.0f, position.y},
                                  {0.0f, 0.0f, scale, position.z}}});
  _scene->add_surface(value.surface);
  value.particle = _space->create_particle(
      {.motion_callback = &value,
       .collision_flags = collision_flags,
       .collision_mask = collision_mask,
       .position = position,
       .velocity = velocity,
       .mass = density * 4.0f / 3.0f * 3.14f * scale * scale * scale,
       .radius = scale,
       .material = {.static_friction_coefficient = 0.3f,
                    .dynamic_friction_coefficient = 0.2f,
                    .restitution_coefficient = 0.1f}});
  return {_next_entity_reference_value++};
}

void Test_entity_manager::destroy_entity(Test_entity_handle handle) {
  auto const it = _entities.find(handle.value);
  auto &value = it->second;
  _space->destroy_particle(value.particle);
  _scene->remove_surface(value.surface);
  _graphics->destroy_surface(value.surface);
  _entities.erase(it);
}

void Test_entity_manager::tick(float delta_time) {
  auto entities_to_destroy = std::vector<Test_entity_handle>{};
  for (auto &[handle, value] : _entities) {
    value.time_alive += delta_time;
    if (value.time_alive > 20.0f) {
      entities_to_destroy.push_back({handle});
    }
  }
  for (auto const handle : entities_to_destroy) {
    destroy_entity(handle);
  }
}

void Test_entity_manager::Entity::on_particle_motion(
    physics::Particle_motion_event const &event) {
  auto transform = surface->get_transform();
  transform[0][3] = event.position.x;
  transform[1][3] = event.position.y;
  transform[2][3] = event.position.z;
  surface->set_transform(transform);
}
} // namespace client
} // namespace marlon