#include <array>
#include <fstream>
#include <iostream>
#include <vector>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#pragma clang diagnostic pop
#else
#include <glad/glad.h>
#endif

#include <GLFW/glfw3.h>

#include "../graphics/gl/graphics.h"
#include "../physics/physics.h"
#include "application_loop.h"
#include "dynamic_prop.h"
#include "glfw_instance.h"
#include "glfw_window.h"
#include "static_prop.h"
#include "test_entity.h"

namespace client = marlon::client;
namespace graphics = marlon::graphics;
namespace math = marlon::math;
namespace physics = marlon::physics;

struct Vertex {
  math::Vec3f position;
  math::Vec2f texcoord;
};

struct Resources {
  graphics::Unique_texture_ptr brick_base_color_texture;
  graphics::Unique_texture_ptr striped_cotton_base_color_texture;
  graphics::Unique_material_ptr brick_material;
  graphics::Unique_material_ptr striped_cotton_material;
  graphics::Unique_material_ptr red_material;
  graphics::Unique_material_ptr blue_material;
  graphics::Unique_material_ptr particle_material;
  graphics::Unique_mesh_ptr cube_mesh;
  graphics::Unique_mesh_ptr low_quality_sphere_mesh;
  graphics::Unique_mesh_ptr high_quality_sphere_mesh;
};

client::Unique_glfw_window_ptr create_window() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  return client::make_unique_glfw_window(1600, 900, "title");
}

std::unique_ptr<graphics::Gl_graphics> create_graphics(GLFWwindow *window) {
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader([](char const *procname) {
        return reinterpret_cast<void *>(glfwGetProcAddress(procname));
      })) {
    throw std::runtime_error{"Failed to initialize OpenGL"};
  }
  return std::make_unique<graphics::Gl_graphics>();
}

graphics::Unique_texture_ptr create_texture(graphics::Graphics *graphics,
                                            const char *path);

graphics::Unique_mesh_ptr create_cuboid_mesh(graphics::Graphics *graphics);

graphics::Unique_mesh_ptr create_icosphere_mesh(graphics::Graphics *graphics,
                                                int subdivisions = 0);

Resources create_resources(graphics::Graphics *graphics) {
  Resources retval;
  retval.brick_base_color_texture = create_texture(
      graphics, "C:/Users/mdron/Sandbox/res/BrickWall29_4K_BaseColor.ktx");
  retval.striped_cotton_base_color_texture = create_texture(
      graphics, "C:/Users/mdron/Sandbox/res/StripedCotton01_2K_BaseColor.ktx");
  retval.brick_material = graphics->create_material_unique(
      {.base_color_texture = retval.brick_base_color_texture.get()});
  retval.striped_cotton_material = graphics->create_material_unique(
      {.base_color_texture = retval.striped_cotton_base_color_texture.get()});
  retval.red_material =
      graphics->create_material_unique({.base_color_tint = {0.1f, 0.0f, 0.0f}});
  retval.blue_material = graphics->create_material_unique(
      {.base_color_tint = {0.00f, 0.005f, 0.02f}});
  retval.particle_material = graphics->create_material_unique(
      {.base_color_tint = {0.25f, 0.5f, 1.0f}});
  retval.cube_mesh = create_cuboid_mesh(graphics);
  retval.low_quality_sphere_mesh = create_icosphere_mesh(graphics, 1);
  retval.high_quality_sphere_mesh = create_icosphere_mesh(graphics, 3);
  return retval;
}

graphics::Unique_texture_ptr create_texture(graphics::Graphics *graphics,
                                            const char *path) {
  std::ifstream in{path, std::ios_base::binary};
  if (in) {
    in.seekg(0, std::ios_base::end);
    auto const file_size = static_cast<std::size_t>(in.tellg());
    auto data = std::vector<unsigned char>();
    data.resize(file_size);
    in.seekg(0, std::ios_base::beg);
    in.read(reinterpret_cast<char *>(data.data()), file_size);
    std::cout << "read " << file_size << " bytes from " << path << std::endl;
    auto retval = graphics->create_texture_unique(
        {.source = graphics::Texture_memory_source{.data = data.data(),
                                                   .size = data.size()}});
    return retval;
  } else {
    throw std::runtime_error{"Failed to open texture file."};
  }
}

graphics::Unique_mesh_ptr create_cuboid_mesh(graphics::Graphics *graphics) {
  std::vector<Vertex> const vertices{// -x face
                                     {{-1.0f, -1.0f, -1.0f}, {0.0f, 0.0f}},
                                     {{-1.0f, -1.0f, 1.0f}, {1.0f, 0.0f}},
                                     {{-1.0f, 1.0f, -1.0f}, {0.0f, 1.0f}},
                                     {{-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f}},
                                     // +x face
                                     {{1.0f, -1.0f, 1.0f}, {0.0f, 0.0f}},
                                     {{1.0f, -1.0f, -1.0f}, {1.0f, 0.0f}},
                                     {{1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}},
                                     {{1.0f, 1.0f, -1.0f}, {1.0f, 1.0f}},
                                     // -y face
                                     {{-1.0f, -1.0f, -1.0f}, {0.0f, 0.0f}},
                                     {{1.0f, -1.0f, -1.0f}, {1.0f, 0.0f}},
                                     {{-1.0f, -1.0f, 1.0f}, {0.0f, 1.0f}},
                                     {{1.0f, -1.0f, 1.0f}, {1.0f, 1.0f}},
                                     // +y face
                                     {{-1.0f, 1.0f, 1.0f}, {0.0f, 0.0f}},
                                     {{1.0f, 1.0f, 1.0f}, {1.0f, 0.0f}},
                                     {{-1.0f, 1.0f, -1.0f}, {0.0f, 1.0f}},
                                     {{1.0f, 1.0f, -1.0f}, {1.0f, 1.0f}},
                                     // -z face
                                     {{1.0f, -1.0f, -1.0f}, {0.0f, 0.0f}},
                                     {{-1.0f, -1.0f, -1.0f}, {1.0f, 0.0f}},
                                     {{1.0f, 1.0f, -1.0f}, {0.0f, 1.0f}},
                                     {{-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f}},
                                     // +z face
                                     {{-1.0f, -1.0f, 1.0f}, {0.0f, 0.0f}},
                                     {{1.0f, -1.0f, 1.0f}, {1.0f, 0.0f}},
                                     {{-1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}},
                                     {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f}}};
  std::vector<std::uint32_t> const indices{
      0,  1,  2,  3,  2,  1,  4,  5,  6,  7,  6,  5,  8,  9,  10, 11, 10, 9,
      12, 13, 14, 15, 14, 13, 16, 17, 18, 19, 18, 17, 20, 21, 22, 23, 22, 21};
  return graphics->create_mesh_unique(
      {.index_format = graphics::Mesh_index_format::uint32,
       .index_count = static_cast<std::uint32_t>(indices.size()),
       .index_data = indices.data(),
       .vertex_format = {.position_fetch_info =
                             {.format =
                                  graphics::Mesh_vertex_position_format::float3,
                              .offset = 0u},
                         .texcoord_fetch_info =
                             {.format =
                                  graphics::Mesh_vertex_texcoord_format::float2,
                              .offset = 12u},
                         .stride = 20u},
       .vertex_count = static_cast<std::uint32_t>(vertices.size()),
       .vertex_data = vertices.data()});
}

graphics::Unique_mesh_ptr create_icosphere_mesh(graphics::Graphics *graphics,
                                                int subdivisions) {
  auto const phi = std::numbers::phi_v<float>;
  std::vector<Vertex> vertices{
      {math::normalize(math::Vec3f{phi, 1.0f, 0.0f}), {}},
      {math::normalize(math::Vec3f{phi, -1.0f, 0.0f}), {}},
      {math::normalize(math::Vec3f{-phi, -1.0f, 0.0f}), {}},
      {math::normalize(math::Vec3f{-phi, 1.0f, 0.0f}), {}},
      {math::normalize(math::Vec3f{1.0f, 0.0f, phi}), {}},
      {math::normalize(math::Vec3f{-1.0f, 0.0f, phi}), {}},
      {math::normalize(math::Vec3f{-1.0f, 0.0f, -phi}), {}},
      {math::normalize(math::Vec3f{1.0f, 0.0f, -phi}), {}},
      {math::normalize(math::Vec3f{0.0f, phi, 1.0f}), {}},
      {math::normalize(math::Vec3f{0.0f, phi, -1.0f}), {}},
      {math::normalize(math::Vec3f{0.0f, -phi, -1.0f}), {}},
      {math::normalize(math::Vec3f{0.0f, -phi, 1.0f}), {}}};
  std::vector<std::uint32_t> indices{
      0, 9, 8,  0, 8,  4,  0,  4, 1, 0, 1, 7,  0, 7,  9,  3, 8,  9,  3, 9,
      6, 3, 6,  2, 3,  2,  5,  3, 5, 8, 4, 8,  5, 11, 1,  4, 11, 4,  5, 11,
      5, 2, 11, 2, 10, 11, 10, 1, 6, 9, 7, 10, 7, 1,  10, 6, 7,  10, 2, 6};
  for (int i = 0; i < subdivisions; ++i) {
    auto const indices_copy = indices;
    indices.clear();
    for (auto j = std::size_t{}; j != indices_copy.size(); j += 3) {
      auto const index_0 = indices_copy[j + 0];
      auto const index_1 = indices_copy[j + 1];
      auto const index_2 = indices_copy[j + 2];
      auto const &vertex_0 = vertices[index_0];
      auto const &vertex_1 = vertices[index_1];
      auto const &vertex_2 = vertices[index_2];
      auto const new_vertex_0 = Vertex{
          math::normalize(0.5f * (vertex_0.position + vertex_1.position)), {}};
      auto const new_vertex_1 = Vertex{
          math::normalize(0.5f * (vertex_1.position + vertex_2.position)), {}};
      auto const new_vertex_2 = Vertex{
          math::normalize(0.5f * (vertex_2.position + vertex_0.position)), {}};
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
                         .texcoord_fetch_info =
                             {.format =
                                  graphics::Mesh_vertex_texcoord_format::float2,
                              .offset = 12u},
                         .stride = 20u},
       .vertex_count = static_cast<std::uint32_t>(vertices.size()),
       .vertex_data = vertices.data()});
}

void run_game_loop(GLFWwindow *window,
                   graphics::Graphics *graphics,
                   graphics::Render_target *render_target,
                   graphics::Scene *scene,
                   graphics::Camera *camera,
                   physics::Space *space,
                   client::Dynamic_prop_manager *cotton_box_manager,
                   client::Test_entity_manager *test_entity_manager) {
  auto const tick_rate = 64.0f;
  auto const tick_duration = 1.0f / tick_rate;
  auto loop =
      client::Application_loop{{.space = space,
                                .physics_step_duration = tick_duration,
                                .physics_substep_count = 32,
                                .min_position_iterations_per_contact = 1,
                                .max_position_iterations_per_contact = 4,
                                .min_velocity_iterations_per_contact = 1,
                                .max_velocity_iterations_per_contact = 4}};
  auto previous_time = glfwGetTime();
  auto fps_time_accumulator = 0.0;
  auto fps_frame_accumulator = 0;
  auto worst_frame_time = 0.0;
  auto box_spawn_timer = 0.0;
  auto height = 0.5f;
  auto direction = 0;
  auto count = 0;
  auto offsetX = 0.0f;
  auto offsetZ = 0.0f;
  auto spawn_debounce = 0.0f;
  for (;;) {
    glfwPollEvents();
    if (glfwWindowShouldClose(window)) {
      break;
    }
    auto const current_time = glfwGetTime();
    auto const elapsed_time = current_time - previous_time;
    previous_time = current_time;
    if (loop.run_once(elapsed_time)) {
      test_entity_manager->tick(tick_duration);
      for (auto i = 0; i < 0; ++i) {
        test_entity_manager->create_entity({});
      }
      if (glfwGetKey(window, GLFW_KEY_SPACE) && spawn_debounce >= 0.0f) {
        spawn_debounce = -0.1f;
        cotton_box_manager->create(
            {.position = math::Vec3f{10.0f, height + 2.0f, 10.0f},
             .velocity = math::Vec3f{-10.0f, 0.0f, -10.0f},
             .orientation = math::Quatf::axis_angle(
                 math::Vec3f{0.0f, 1.0f, 0.0f},
                 math::deg_to_rad(direction == 0 ? 90.0f : 0.0f)),
             .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f}});
      }
    }
    spawn_debounce += elapsed_time;
    box_spawn_timer += elapsed_time;
    if (box_spawn_timer > 1.25f) {
      box_spawn_timer = 0.0f;
      if (height > 8.0f) {
        height = 2.0f;
        offsetX = 10 * (rand() / (float)RAND_MAX) - 5;
        offsetZ = 10 * (rand() / (float)RAND_MAX) - 5;
      } else {
        height += 0.6f;
      }
      ++count;
      if (count == 128) {
        box_spawn_timer = -1000.0f;
      }
      std::cout << "box count: " << count << "\n";
      // if (count >= 3) {
      //   direction = 1 - direction;
      //   count = 0;
      //   height += 0.25f;
      // }
      // cotton_box_manager->create(
      //     {.position = math::Vec3f{direction == 0 ? 0.0f : -0.65f * count +
      //     0.65f, height, direction == 0 ? 0.65f * count : 0.65f},
      //      .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
      //      .orientation = math::Quatf::axis_angle(
      //          math::Vec3f{0.0f, 1.0f, 0.0f},
      //          math::deg_to_rad(direction == 0 ? 90.0f : 0.0f)),
      //      .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f}});
      cotton_box_manager->create(
          {.position = math::Vec3f{offsetX, height, offsetZ},
           .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
           .orientation = math::Quatf::axis_angle(
               math::Vec3f{0.0f, 1.0f, 0.0f},
               math::deg_to_rad(direction == 0 ? 90.0f : 0.0f)),
           .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f}});
      // count += 1;
    }
    graphics->render(scene, camera, render_target);
    glfwSwapBuffers(window);
    fps_time_accumulator += elapsed_time;
    ++fps_frame_accumulator;
    if (elapsed_time > worst_frame_time) {
      worst_frame_time = elapsed_time;
    }
    while (fps_time_accumulator >= 1.0) {
      std::cout << "frames per second: " << fps_frame_accumulator << std::endl;
      std::cout << "worst frame time: " << worst_frame_time << std::endl;
      std::cout << "running for " << glfwGetTime() << " seconds" << std::endl;
      fps_time_accumulator -= 1.0;
      fps_frame_accumulator = 0;
      worst_frame_time = 0.0;
    }
  }
}

int main() {
  client::Shared_glfw_instance const glfw;
  auto const window = create_window();
  auto const graphics = create_graphics(window.get());
  glfwSwapInterval(0);
  auto const resources = create_resources(graphics.get());
  auto const scene = graphics->create_scene_unique({});
  auto const camera = graphics->create_camera_unique(
      {.zoom = math::Vec2f{9.0f / 16.0f, 1.0f} * 2.0f,
       .near_plane_distance = 0.01f,
       .far_plane_distance = 1000.0f,
       .position = {-10.0f, 3.5f, 10.0f},
       .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                              math::deg_to_rad(-45.0f)) *
                      math::Quatf::axis_angle(math::Vec3f{1.0f, 0.0f, 0.0f},
                                              math::deg_to_rad(-12.0f))});
  auto const ground_surface = graphics->create_surface_unique(
      {.mesh = resources.cube_mesh.get(),
       .material = resources.blue_material.get(),
       .transform = math::Mat3x4f{{100.0f, 0.0f, 0.0f, 0.0f},
                                  {0.0f, 100.0f, 0.0f, -100.0f},
                                  {0.0f, 0.0f, 100.0f, 0.0f}}});
  scene->add_surface(ground_surface.get());
  physics::Space space{{.gravitational_acceleration = {0.0f, -9.8f, 0.0f}}};
  physics::Material const physics_material{.static_friction_coefficient = 0.4f,
                                           .dynamic_friction_coefficient = 0.3f,
                                           .restitution_coefficient = 0.1f};
  physics::Box ground_shape{{100.0f, 0.5f, 100.0f}};
  physics::Ball ball_shape{0.5f};
  physics::Box brick_box_shape{{1.0f, 1.0f, 1.0f}};
  physics::Box cotton_box_shape{{0.3f, 0.3f, 0.3f}};
  space.create_static_rigid_body({.shape = ground_shape,
                                  .material = physics_material,
                                  .position = math::Vec3f{0.0f, -0.5f, 0.0f}});
  client::Static_prop_manager red_ball_manager{
      {.graphics = graphics.get(),
       .scene = scene.get(),
       .surface_mesh = resources.high_quality_sphere_mesh.get(),
       .surface_material = resources.red_material.get(),
       .surface_pretransform = math::Mat3x4f{{0.5f, 0.0f, 0.0f, 0.0f},
                                             {0.0f, 0.5f, 0.0f, 0.0f},
                                             {0.0f, 0.0f, 0.5f, 0.0f}},
       .space = &space,
       .body_shape = ball_shape,
       .body_material = {.static_friction_coefficient = 0.2f,
                         .dynamic_friction_coefficient = 0.1f,
                         .restitution_coefficient = 0.3f}}};
  client::Static_prop_manager brick_box_manager{
      {.graphics = graphics.get(),
       .scene = scene.get(),
       .surface_mesh = resources.cube_mesh.get(),
       .surface_material = resources.brick_material.get(),
       .space = &space,
       .body_shape = brick_box_shape,
       .body_material = physics_material}};
  client::Dynamic_prop_manager cotton_box_manager{
      {.graphics = graphics.get(),
       .scene = scene.get(),
       .surface_mesh = resources.cube_mesh.get(),
       .surface_material = resources.striped_cotton_material.get(),
       .surface_pretransform =
           math::Mat3x4f{{cotton_box_shape.half_extents[0], 0.0f, 0.0f, 0.0f},
                         {0.0f, cotton_box_shape.half_extents[1], 0.0f, 0.0f},
                         {0.0f, 0.0f, cotton_box_shape.half_extents[2], 0.0f}},
       .space = &space,
       .body_mass = 80.0f,
       .body_inertia_tensor =
           80.0f * physics::solid_inertia_tensor(cotton_box_shape),
       .body_shape = cotton_box_shape,
       .body_material = {.static_friction_coefficient = 0.3f,
                         .dynamic_friction_coefficient = 0.2f,
                         .restitution_coefficient = 0.2f}}};
  client::Test_entity_manager test_entity_manager{
      {.graphics = graphics.get(),
       .scene = scene.get(),
       .surface_mesh = resources.low_quality_sphere_mesh.get(),
       .surface_material = resources.particle_material.get(),
       .space = &space}};
  red_ball_manager.create({.position = {-1.5f, 0.5f, -1.5f}});
  red_ball_manager.create({.position = {1.5f, 0.5f, 1.5f}});
  // brick_box_manager.create(
  //     {.position = {0.0f, 10.0f, 0.0f},
  //      .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
  //                                             math::deg_to_rad(-45.0f)) *
  //                     math::Quatf::axis_angle(math::Vec3f{0.0f, 0.0f, 1.0f},
  //                                             math::deg_to_rad(45.0f))});
  // cotton_box_manager.create(
  //     {.position = math::Vec3f{0.0f, 1.5f, 0.0f},
  //      .angular_velocity = math::Vec3f{0.0f, 10.0f, 0.0f}});
  // cotton_box_manager.create(
  //     {.position = math::Vec3f{0.0f, 3.5f, 0.0f},
  //      .velocity = math::Vec3f{0.0f, 7.0f, 0.0f},
  //      .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f}});
  // cotton_box_manager.create(
  //     {.position = math::Vec3f{3.0f, 1.5f, 3.0f},
  //      .velocity = math::Vec3f{0.0f, 7.0f, 0.0f},
  //      .orientation = math::Quatf::axis_angle(math::Vec3f{1.0f, 0.0f, 0.0f},
  //                                             math::deg_to_rad(90.0f)),
  //      .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f}});
  // cotton_box_manager.create(
  //     {.position = math::Vec3f{-3.0f, 1.5f, -3.0f},
  //      .velocity = math::Vec3f{0.0f, 7.0f, 0.0f},
  //      .orientation = math::Quatf::axis_angle(math::Vec3f{1.0f, 0.0f, 0.0f},
  //                                             math::deg_to_rad(90.0f)),
  //      .angular_velocity = math::Vec3f{0.0f, 20.0f, 0.0f}});
  // graphics->apply_scene_diff(scene_diff.get());
  auto const render_target = graphics->get_default_render_target();
  run_game_loop(window.get(),
                graphics.get(),
                render_target,
                scene.get(),
                camera.get(),
                &space,
                &cotton_box_manager,
                &test_entity_manager);
  return 0;
}