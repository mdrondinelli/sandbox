#include "test_entity.h"

namespace marlon {
namespace client {
Test_entity_manager::Test_entity_manager(
    graphics::Graphics *graphics, graphics::Scene_diff *const *scene_diff,
    physics::Space *space, Entity_construction_queue *entity_construction_queue,
    Entity_destruction_queue *entity_destruction_queue)
    : _graphics{graphics}, _scene_diff{scene_diff}, _space{space},
      _entity_construction_queue{entity_construction_queue},
      _entity_destruction_queue{entity_destruction_queue}, _material{nullptr},
      _mesh{nullptr}, _surface{nullptr},
      _random_number_engine{std::random_device{}()},
      _next_entity_reference_value{} {
  std::vector<math::Vec3f> const vertices{
      {-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, -1.0f},
      {-1.0f, 1.0f, 1.0f},   {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f},
      {1.0f, 1.0f, -1.0f},   {1.0f, 1.0f, 1.0f}};
  std::vector<std::uint32_t> const indices{0, 1, 2, 3, 2, 1, 1, 5, 3, 7, 3, 5,
                                           5, 4, 7, 6, 7, 4, 4, 0, 6, 2, 6, 0,
                                           0, 4, 1, 5, 1, 4, 3, 7, 2, 6, 2, 7};
  auto material =
      graphics->create_material_unique({.albedo = {0.0f, 0.5f, 1.0f}});
  auto mesh = graphics->create_mesh_unique(
      {.index_format = graphics::Mesh_index_format::uint32,
       .index_count = static_cast<std::uint32_t>(indices.size()),
       .index_data = indices.data(),
       .vertex_format = {.position_fetch_info =
                             {.format =
                                  graphics::Mesh_vertex_position_format::float3,
                              .offset = 0u},
                         .stride = 12u},
       .vertex_count = static_cast<std::uint32_t>(vertices.size()),
       .vertex_data = vertices.data()});
  auto surface = graphics->create_surface_unique(
      {.material = material.get(), .mesh = mesh.get()});
  _material = material.release();
  _mesh = mesh.release();
  _surface = surface.release();
}

Test_entity_manager::~Test_entity_manager() {
  _graphics->destroy_surface(_surface);
  _graphics->destroy_mesh(_mesh);
  _graphics->destroy_material(_material);
}

namespace {
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
  std::uniform_real_distribution<float> scale_distribution{0.05f, 0.1f};
//   std::uniform_real_distribution<float> horizontal_angle_distribution{
//       0.0f, 2.0f * std::numbers::pi_v<float>};
//   std::uniform_real_distribution<float> vertical_angle_distribution{
//       math::deg_to_rad(-90.0f), math::deg_to_rad(0.0f)};
//   std::exponential_distribution<float> speed_distribution{2.0f};
  auto const position =
      sample_ball(_random_number_engine) * 0.5f + math::Vec3f{0.0f, 4.5f, 0.0f};
  auto const velocity = sample_ball(_random_number_engine);
  auto const scale = scale_distribution(_random_number_engine);
//   auto const horizontal_angle =
//       horizontal_angle_distribution(_random_number_engine);
//   auto const vertical_angle =
//       vertical_angle_distribution(_random_number_engine);
//   auto const speed = 1.0f * speed_distribution(_random_number_engine);
  Entity_reference const reference{_next_entity_reference_value};
  auto &value = _entities[reference];
  value.manager = this;
  value.reference = reference;
  value.scene_node = _graphics->record_scene_node_creation(
      *_scene_diff, {.translation = position, .scale = scale});
  value.surface_instance = _graphics->record_surface_instance_creation(
      *_scene_diff, {.surface = _surface, .scene_node = value.scene_node});
  value.particle = _space->create_particle({.position = position,
                                            .velocity = velocity,
                                            .mass = 0.5f,
                                            .motion_callback = &value});
  ++_next_entity_reference_value;
  return reference;
}

void Test_entity_manager::destroy_entity(Entity_reference reference) {
  auto const it = _entities.find(reference);
  auto &value = it->second;
  _graphics->record_surface_instance_destruction(*_scene_diff,
                                                 value.surface_instance);
  _graphics->record_scene_node_destruction(*_scene_diff, value.scene_node);
  _space->destroy_particle(value.particle);
  _entities.erase(it);
}

void Test_entity_manager::Test_entity::on_particle_motion(
    physics::Particle_motion_event const &event) {
  manager->_graphics->record_scene_node_translation_continuous(
      *manager->_scene_diff, scene_node, event.position);
  if (event.position.y < 0.0f) {
    manager->_entity_destruction_queue->push(manager, reference);
    manager->_entity_construction_queue->push(manager, {});
  }
}
} // namespace client
} // namespace marlon