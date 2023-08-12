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
  graphics::Unique_surface_ptr brick_box_surface;
  graphics::Unique_surface_ptr striped_cotton_box_surface;
  graphics::Unique_surface_ptr red_ball_surface;
  graphics::Unique_surface_ptr blue_box_surface;
  graphics::Unique_surface_ptr particle_surface;
};

client::Unique_glfw_window_ptr create_window() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  return client::make_unique_glfw_window(1280, 720, "title");
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
      graphics,
      "C:/Users/marlo/rendering-engine/res/BrickWall29_4K_BaseColor.ktx");
  retval.striped_cotton_base_color_texture = create_texture(
      graphics,
      "C:/Users/marlo/rendering-engine/res/StripedCotton01_2K_BaseColor.ktx");
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
  retval.brick_box_surface =
      graphics->create_surface_unique({.material = retval.brick_material.get(),
                                       .mesh = retval.cube_mesh.get()});
  retval.striped_cotton_box_surface = graphics->create_surface_unique(
      {.material = retval.striped_cotton_material.get(),
       .mesh = retval.cube_mesh.get()});
  retval.red_ball_surface = graphics->create_surface_unique(
      {.material = retval.red_material.get(),
       .mesh = retval.high_quality_sphere_mesh.get()});
  retval.blue_box_surface = graphics->create_surface_unique(
      {.material = retval.blue_material.get(), .mesh = retval.cube_mesh.get()});
  retval.particle_surface = graphics->create_surface_unique(
      {.material = retval.particle_material.get(),
       .mesh = retval.low_quality_sphere_mesh.get()});
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

void tick(physics::Space *space,
          client::Test_entity_manager *test_entity_manager, float delta_time);

void run_game_loop(GLFWwindow *window, graphics::Graphics *graphics,
                   graphics::Render_target *render_target,
                   graphics::Scene *scene, graphics::Scene_diff *scene_diff,
                   graphics::Camera_instance *camera_instance,
                   physics::Space *space,
                   client::Test_entity_manager *test_entity_manager) {
  auto const tick_rate = 30.0;
  auto const tick_duration = 1.0 / tick_rate;
  auto previous_time = glfwGetTime();
  auto accumulated_time = 0.0;
  auto fps_time_accumulator = 0.0;
  auto fps_frame_accumulator = 0;
  auto worst_frame_time = 0.0;
  for (;;) {
    glfwPollEvents();
    if (glfwWindowShouldClose(window)) {
      break;
    }
    auto const current_time = glfwGetTime();
    auto const elapsed_time = current_time - previous_time;
    previous_time = current_time;
    accumulated_time += elapsed_time;
    if (accumulated_time >= tick_duration) {
      do {
        accumulated_time -= tick_duration;
        graphics->apply_scene_diff(scene_diff);
        tick(space, test_entity_manager, static_cast<float>(tick_duration));
      } while (accumulated_time >= tick_duration);
    }
    graphics->apply_scene_diff(
        scene_diff,
        static_cast<float>(elapsed_time /
                           (elapsed_time + tick_duration - accumulated_time)));
    graphics->render(scene, camera_instance, render_target);
    glfwSwapBuffers(window);
    fps_time_accumulator += elapsed_time;
    ++fps_frame_accumulator;
    if (elapsed_time > worst_frame_time) {
      worst_frame_time = elapsed_time;
    }
    while (fps_time_accumulator >= 1.0) {
      std::cout << "frames per second: " << fps_frame_accumulator << std::endl;
      std::cout << "worst frame time: " << worst_frame_time << std::endl;
      fps_time_accumulator -= 1.0;
      fps_frame_accumulator = 0;
      worst_frame_time = 0.0;
    }
  }
}

void tick(physics::Space *space,
          client::Test_entity_manager *test_entity_manager, float delta_time) {
  space->simulate({.delta_time = delta_time, .substep_count = 2});
  test_entity_manager->tick(delta_time);
  for (auto i = 0; i < 8; ++i) {
    test_entity_manager->create_entity({});
  }
}

int main() {
  client::Shared_glfw_instance const glfw;
  auto const window = create_window();
  auto const graphics = create_graphics(window.get());
  glfwSwapInterval(0);
  auto const resources = create_resources(graphics.get());
  auto const scene = graphics->create_scene_unique({});
  auto const scene_diff =
      graphics->create_scene_diff_unique({.scene = scene.get()});
  class Scene_diff_provider : public client::Scene_diff_provider {
  public:
    explicit Scene_diff_provider(graphics::Scene_diff *scene_diff) noexcept
        : _scene_diff{scene_diff} {}

    graphics::Scene_diff *get_scene_diff() const noexcept final {
      return _scene_diff;
    };

  private:
    graphics::Scene_diff *_scene_diff;
  };
  auto const scene_diff_provider = Scene_diff_provider{scene_diff.get()};
  auto const camera = scene_diff->record_camera_creation({
      .near_plane_distance = 0.001f,
      .far_plane_distance = 1000.0f,
      .zoom_x = 9.0f / 16.0f,
      .zoom_y = 1.0f,
  });
  auto const camera_scene_node = scene_diff->record_scene_node_creation(
      {.translation = {-3.0f, 3.5f, 3.0f},
       .rotation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                           math::deg_to_rad(-45.0f)) *
                   math::Quatf::axis_angle(math::Vec3f{1.0f, 0.0f, 0.0f},
                                           math::deg_to_rad(-30.0f))});
  auto const camera_instance = scene_diff->record_camera_instance_creation(
      {.camera = camera, .scene_node = camera_scene_node});
  auto const ground_scene_node = scene_diff->record_scene_node_creation(
      {.translation = {0.0f, -100.0f, 0.0f}, .scale = 100.0f});
  scene_diff->record_surface_instance_creation(
      {.surface = resources.blue_box_surface.get(),
       .scene_node = ground_scene_node});
  physics::Space space{{.gravitational_acceleration = {0.0f, -9.8f, 0.0f}}};
  physics::Material const physics_material{.static_friction_coefficient = 1.1f,
                                           .dynamic_friction_coefficient = 0.6f,
                                           .restitution_coefficient = 0.0f};
  physics::Box ground_shape{50.0f, 0.5f, 50.0f};
  physics::Ball ball_shape{0.5f};
  physics::Box brick_box_shape{1.0f, 1.0f, 1.0f};
  physics::Box cotton_box_shape{0.5f, 0.5f, 0.5f};
  space.create_static_rigid_body({.collision_flags = 1,
                                  .collision_mask = 1,
                                  .position = math::Vec3f{0.0f, -0.5f, 0.0f},
                                  // .orientation = math::Quatf::identity(),
                                  .shape = ground_shape,
                                  .material = physics_material});
  client::Static_prop_manager red_ball_manager{
      {.scene_diff_provider = &scene_diff_provider,
       .surface = resources.red_ball_surface.get(),
       .surface_scale = 0.5f,
       .space = &space,
       .shape = ball_shape,
       .material = {.static_friction_coefficient = 1.1f,
                    .dynamic_friction_coefficient = 0.75f,
                    .restitution_coefficient = 0.0f}}};
  client::Static_prop_manager brick_box_manager{
      {.scene_diff_provider = &scene_diff_provider,
       .surface = resources.brick_box_surface.get(),
       .surface_scale = 1.0f,
       .space = &space,
       .shape = brick_box_shape,
       .material = physics_material}};
  client::Dynamic_prop_manager cotton_box_manager{
      {.scene_diff_provider = &scene_diff_provider,
       .surface = resources.striped_cotton_box_surface.get(),
       .surface_scale = 0.5f,
       .space = &space,
       .mass = 1.0f,
       .inertia_tensor = math::Mat3x3f::identity(),
       .shape = cotton_box_shape,
       .material = physics_material}};
  client::Test_entity_manager test_entity_manager{
      &scene_diff_provider, resources.particle_surface.get(), &space};
  red_ball_manager.create({.position = {-1.5f, 0.5f, -1.5f}});
  red_ball_manager.create({.position = {1.5f, 0.5f, 1.5f}});
  brick_box_manager.create(
      {.position = {0.0f, 1.5f, 0.0f},
       .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                              math::deg_to_rad(-45.0f)) *
                      math::Quatf::axis_angle(math::Vec3f{0.0f, 0.0f, 1.0f},
                                              math::deg_to_rad(45.0f))});
  cotton_box_manager.create(
      {.position = math::Vec3f{0.0f, 5.0f, 0.0f},
       .angular_velocity = math::Vec3f{0.0f, 1.0f, 0.0f}});
  graphics->apply_scene_diff(scene_diff.get());
  auto const render_target = graphics->get_default_render_target();
  run_game_loop(window.get(), graphics.get(), render_target, scene.get(),
                scene_diff.get(), camera_instance, &space,
                &test_entity_manager);
  return 0;
}