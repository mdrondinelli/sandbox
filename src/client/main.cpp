#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "../graphics/gl/graphics.h"
#include "../physics/space.h"
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
  return client::make_glfw_unique_window(1600, 900, "title");
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

void tick(physics::Space *space,
          client::Entity_construction_queue *entity_construction_queue,
          client::Entity_destruction_queue *entity_destruction_queue,
          client::Test_entity_manager *test_entity_manager, float delta_time) {
  static int tick_number = 0;
  if (tick_number++ < 400) {
    for (int i = 0; i < 10; ++i) {
      test_entity_manager->create_entity({});
    }
  }
  entity_construction_queue->consume();
  space->simulate({.acceleration = {0.0, -9.8f, 0.0f},
                   .delta_time = delta_time,
                   .substep_count = 1});
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
  auto const tick_rate = 16.0;
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
  auto const ground_material =
      graphics->create_material_unique({.albedo = {0.5f, 0.5f, 0.5f}});
  auto const ground_mesh = create_cube_mesh_unique(graphics.get());
  auto const ground_surface = graphics->create_surface_unique(
      {.material = ground_material.get(), .mesh = ground_mesh.get()});
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
  physics::Space space;
  client::Entity_construction_queue entity_construction_queue;
  client::Entity_destruction_queue entity_destruction_queue;
  client::Test_entity_manager test_entity_manager{
      graphics.get(), &scene_diff_raw, &space, &entity_construction_queue,
      &entity_destruction_queue};
  graphics->apply_scene_diff(scene_diff_raw);
  auto const render_target = graphics->get_default_render_target();
  run_game_loop(window.get(), graphics.get(), render_target, scene.get(),
                scene_diff_raw, camera_instance, &space,
                &entity_construction_queue, &entity_destruction_queue,
                &test_entity_manager);
  return 0;
}