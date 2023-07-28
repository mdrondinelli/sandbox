#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "../graphics/gl/graphics.h"
#include "../physics/physics.h"
#include "glfw_instance.h"
#include "glfw_window.h"
#include "test_entity.h"

namespace client = marlon::client;
namespace graphics = marlon::graphics;
namespace math = marlon::math;
namespace physics = marlon::physics;

client::Glfw_unique_window_ptr create_window_unique() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  return client::make_glfw_unique_window(1920, 1080, "title");
}

std::unique_ptr<graphics::Gl_graphics>
create_graphics_unique(GLFWwindow *window) {
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader([](char const *procname) {
        return static_cast<void *>(glfwGetProcAddress(procname));
      })) {
    throw std::runtime_error{"Failed to initialize OpenGL"};
  }
  return std::make_unique<graphics::Gl_graphics>();
}

graphics::Unique_mesh_ptr
create_cube_mesh_unique(graphics::Graphics *graphics) {
  std::vector<math::Vec3f> const vertices{
      {-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, -1.0f},
      {-1.0f, 1.0f, 1.0f},   {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f},
      {1.0f, 1.0f, -1.0f},   {1.0f, 1.0f, 1.0f}};
  std::vector<std::uint32_t> const indices{0, 1, 2, 3, 2, 1, 1, 5, 3, 7, 3, 5,
                                           5, 4, 7, 6, 7, 4, 4, 0, 6, 2, 6, 0,
                                           0, 4, 1, 5, 1, 4, 3, 7, 2, 6, 2, 7};
  return graphics->create_mesh_unique(
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
}

graphics::Unique_mesh_ptr
create_icosahedron_mesh_unique(graphics::Graphics *graphics,
                               int subdivisions = 0) {
  auto const phi = std::numbers::phi_v<float>;
  std::vector<math::Vec3f> vertices{
      math::normalize(math::Vec3f{phi, 1.0f, 0.0f}),
      math::normalize(math::Vec3f{phi, -1.0f, 0.0f}),
      math::normalize(math::Vec3f{-phi, -1.0f, 0.0f}),
      math::normalize(math::Vec3f{-phi, 1.0f, 0.0f}),
      math::normalize(math::Vec3f{1.0f, 0.0f, phi}),
      math::normalize(math::Vec3f{-1.0f, 0.0f, phi}),
      math::normalize(math::Vec3f{-1.0f, 0.0f, -phi}),
      math::normalize(math::Vec3f{1.0f, 0.0f, -phi}),
      math::normalize(math::Vec3f{0.0f, phi, 1.0f}),
      math::normalize(math::Vec3f{0.0f, phi, -1.0f}),
      math::normalize(math::Vec3f{0.0f, -phi, -1.0f}),
      math::normalize(math::Vec3f{0.0f, -phi, 1.0f})};
  std::vector<std::uint32_t> indices{
      0, 9, 8,  0, 8,  4,  0,  4, 1, 0, 1, 7,  0, 7,  9,  3, 8,  9,  3, 9,
      6, 3, 6,  2, 3,  2,  5,  3, 5, 8, 4, 8,  5, 11, 1,  4, 11, 4,  5, 11,
      5, 2, 11, 2, 10, 11, 10, 1, 6, 9, 7, 10, 7, 1,  10, 6, 7,  10, 2, 6};
  for (int i = 0; i < subdivisions; ++i) {
    auto const indices_copy = indices;
    indices.clear();
    for (int j = 0; j != indices_copy.size(); j += 3) {
      auto const index_0 = indices_copy[j + 0];
      auto const index_1 = indices_copy[j + 1];
      auto const index_2 = indices_copy[j + 2];
      auto const &vertex_0 = vertices[index_0];
      auto const &vertex_1 = vertices[index_1];
      auto const &vertex_2 = vertices[index_2];
      auto const new_vertex_0 = math::normalize(0.5f * (vertex_0 + vertex_1));
      auto const new_vertex_1 = math::normalize(0.5f * (vertex_1 + vertex_2));
      auto const new_vertex_2 = math::normalize(0.5f * (vertex_2 + vertex_0));
      auto const new_vertex_0_index =
          static_cast<std::uint32_t>(vertices.size());
      auto const new_vertex_1_index =
          static_cast<std::uint32_t>(vertices.size() + 1);
      auto const new_vertex_2_index =
          static_cast<std::uint32_t>(vertices.size() + 2);
      vertices.emplace_back(new_vertex_0);
      vertices.emplace_back(new_vertex_1);
      vertices.emplace_back(new_vertex_2);
      indices.emplace_back(index_0);
      indices.emplace_back(new_vertex_0_index);
      indices.emplace_back(new_vertex_2_index);
      indices.emplace_back(index_1);
      indices.emplace_back(new_vertex_1_index);
      indices.emplace_back(new_vertex_0_index);
      indices.emplace_back(index_2);
      indices.emplace_back(new_vertex_2_index);
      indices.emplace_back(new_vertex_1_index);
      indices.emplace_back(new_vertex_0_index);
      indices.emplace_back(new_vertex_1_index);
      indices.emplace_back(new_vertex_2_index);
    }
  }
  return graphics->create_mesh_unique(
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
}

void tick(physics::Space *space,
          client::Entity_construction_queue *entity_construction_queue,
          client::Entity_destruction_queue *entity_destruction_queue,
          client::Test_entity_manager *test_entity_manager, float delta_time) {
  for (int i = 0; i < 8; ++i)
    entity_construction_queue->push(test_entity_manager, {});
  test_entity_manager->tick_entities(delta_time);
  space->simulate({.delta_time = delta_time, .substep_count = 1});
  entity_construction_queue->consume();
  entity_destruction_queue->consume();
}

void run_game_loop(GLFWwindow *window, graphics::Graphics *graphics,
                   graphics::Render_target *render_target,
                   graphics::Scene *scene, graphics::Scene_diff *scene_diff,
                   graphics::Camera_instance *camera_instance,
                   physics::Space *space,
                   client::Entity_construction_queue *entity_construction_queue,
                   client::Entity_destruction_queue *entity_destruction_queue,
                   client::Test_entity_manager *test_entity_manager) {
  auto const tick_rate = 32.0;
  auto const tick_duration = 1.0 / tick_rate;
  auto previous_time = glfwGetTime();
  auto accumulator = 0.0;
  auto fps_time_accumulator = 0.0;
  auto fps_frame_accumulator = 0;
  // graphics::Unique_scene_diff_ptr scene_diff{nullptr, graphics};
  for (;;) {
    glfwPollEvents();
    if (glfwWindowShouldClose(window)) {
      break;
    }
    auto const current_time = glfwGetTime();
    auto const elapsed_time = current_time - previous_time;
    previous_time = current_time;
    accumulator += elapsed_time;
    if (accumulator >= tick_duration) {
      do {
        accumulator -= tick_duration;
        // if (scene_diff.get() != nullptr) {
        //   graphics->apply_scene_diff(scene_diff.get());
        // }
        // scene_diff =
        graphics->apply_scene_diff(scene_diff);
        tick(space, entity_construction_queue, entity_destruction_queue,
             test_entity_manager, tick_duration);
      } while (accumulator >= tick_duration);
    }
    graphics->apply_scene_diff(
        scene_diff,
        static_cast<float>(elapsed_time /
                           (elapsed_time + tick_duration - accumulator)));
    // else if (scene_diff.get() != nullptr) {
    //   graphics->apply_scene_diff(
    //       scene_diff.get(),
    //       static_cast<float>(elapsed_time /
    //                          (elapsed_time + tick_duration - accumulator)));
    // }
    graphics->render(scene, camera_instance, render_target);
    glfwSwapBuffers(window);
    fps_time_accumulator += elapsed_time;
    ++fps_frame_accumulator;
    while (fps_time_accumulator >= 1.0) {
      std::cout << fps_frame_accumulator << " fps" << std::endl;
      fps_time_accumulator -= 1.0;
      fps_frame_accumulator = 0;
    }
  }
}

int main() {
  client::Glfw_shared_instance const glfw;
  auto const window = create_window_unique();
  auto const graphics = create_graphics_unique(window.get());
  glfwSwapInterval(0);
  auto const particle_material =
      graphics->create_material_unique({.albedo = {0.0f, 0.375f, 1.0f}});
  auto const ground_material =
      graphics->create_material_unique({.albedo = {0.00f, 0.005f, 0.02f}});
  auto const ball_material =
      graphics->create_material_unique({.albedo = {0.2f, 0.0f, 0.0f}});
  auto const cube_mesh = create_cube_mesh_unique(graphics.get());
  auto const icosahedron_mesh = create_icosahedron_mesh_unique(graphics.get());
  auto const sphere_mesh = create_icosahedron_mesh_unique(graphics.get(), 2);
  auto const particle_surface = graphics->create_surface_unique(
      {.material = particle_material.get(), .mesh = icosahedron_mesh.get()});
  auto const ground_surface = graphics->create_surface_unique(
      {.material = ground_material.get(), .mesh = cube_mesh.get()});
  auto const ball_surface = graphics->create_surface_unique(
      {.material = ball_material.get(), .mesh = sphere_mesh.get()});
  auto const box_surface = graphics->create_surface_unique(
      {.material = ball_material.get(), .mesh = cube_mesh.get()});
  auto const scene = graphics->create_scene_unique({});
  auto const scene_diff =
      graphics->create_scene_diff_unique({.scene = scene.get()});
  auto const scene_diff_raw = scene_diff.get();
  auto const camera = graphics->record_camera_creation(
      scene_diff_raw, {
                          .near_plane_distance = 0.001f,
                          .far_plane_distance = 1000.0f,
                          .zoom_x = 9.0f / 16.0f,
                          .zoom_y = 1.0f,
                      });
  auto const camera_scene_node = graphics->record_scene_node_creation(
      scene_diff_raw, {.translation = {0.0f, 1.5f, 8.0f}});
  auto const camera_instance = graphics->record_camera_instance_creation(
      scene_diff_raw, {.camera = camera, .scene_node = camera_scene_node});
  auto const ground_scene_node = graphics->record_scene_node_creation(
      scene_diff_raw, {.translation = {0.0f, -100.0f, 0.0f}, .scale = 100.0f});
  graphics->record_surface_instance_creation(
      scene_diff_raw,
      {.surface = ground_surface.get(), .scene_node = ground_scene_node});
  auto const left_ball_scene_node = graphics->record_scene_node_creation(
      scene_diff_raw, {.translation = {-1.0f, 0.5f, -1.0f}, .scale = 0.5f});
  graphics->record_surface_instance_creation(
      scene_diff_raw,
      {.surface = ball_surface.get(), .scene_node = left_ball_scene_node});
  auto const box_scene_node = graphics->record_scene_node_creation(
      scene_diff_raw,
      {.translation = {0.0f, 1.5f, 0.0f},
       .rotation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                           math::deg_to_rad(-45.0f)) *
                   math::Quatf::axis_angle(math::Vec3f{0.0f, 0.0f, 1.0f},
                                           math::deg_to_rad(45.0f)),
       .scale = 0.5f});
  graphics->record_surface_instance_creation(
      scene_diff_raw,
      {.surface = box_surface.get(), .scene_node = box_scene_node});
  auto const right_ball_scene_node = graphics->record_scene_node_creation(
      scene_diff_raw, {.translation = {1.0f, 0.5f, 1.0f}, .scale = 0.5f});
  graphics->record_surface_instance_creation(
      scene_diff_raw,
      {.surface = ball_surface.get(), .scene_node = right_ball_scene_node});
  physics::Space space;
  physics::Half_space ground_shape{math::Vec3f{0.0f, 1.0f, 0.0f}};
  physics::Ball ball_shape{0.5f};
  physics::Box box_shape{0.5f, 0.5f, 0.5f};
  space.create_static_rigid_body({.collision_flags = 1,
                                  .collision_mask = 1,
                                  .position = math::Vec3f::zero(),
                                  .orientation = math::Quatf::identity(),
                                  .shape = &ground_shape});
  space.create_static_rigid_body({.collision_flags = 1,
                                  .collision_mask = 1,
                                  .position = {-1.0f, 0.5f, -1.0f},
                                  .shape = &ball_shape});
  space.create_static_rigid_body(
      {.collision_flags = 1,
       .collision_mask = 1,
       .position = {0.0f, 1.5f, 0.0f},
       .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                              math::deg_to_rad(-45.0f)) *
                      math::Quatf::axis_angle(math::Vec3f{0.0f, 0.0f, 1.0f},
                                              math::deg_to_rad(45.0f)),
       .shape = &box_shape});
  space.create_static_rigid_body({.collision_flags = 1,
                                  .collision_mask = 1,
                                  .position = {1.0f, 0.5f, 1.0f},
                                  .shape = &ball_shape});
  client::Entity_construction_queue entity_construction_queue;
  client::Entity_destruction_queue entity_destruction_queue;
  client::Test_entity_manager test_entity_manager{graphics.get(),
                                                  &scene_diff_raw,
                                                  particle_surface.get(),
                                                  &space,
                                                  &entity_construction_queue,
                                                  &entity_destruction_queue};
  graphics->apply_scene_diff(scene_diff_raw);
  auto const render_target = graphics->get_default_render_target();
  run_game_loop(window.get(), graphics.get(), render_target, scene.get(),
                scene_diff_raw, camera_instance, &space,
                &entity_construction_queue, &entity_destruction_queue,
                &test_entity_manager);
  return 0;
}