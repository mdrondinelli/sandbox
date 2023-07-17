#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "glfw/instance.h"
#include "glfw/window.h"
#include "rendering/gl/render_engine.h"

using namespace marlon;

int main() {
  glfw::Instance glfw;
  // glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  const auto window = glfw::make_unique_window(800, 600, "title");
  glfwMakeContextCurrent(window.get());
  if (!gladLoadGLLoader([](char const *procname) {
        return static_cast<void *>(glfwGetProcAddress(procname));
      })) {
    std::cerr << "Failed to initialize OpenGL" << std::endl;
    return -1;
  }
  glfwSwapInterval(1);
  auto const render_engine = std::make_unique<rendering::Gl_render_engine>();
  auto const material =
      render_engine->create_material({.albedo = {1.0f, 1.0f, 1.0f}});
  auto const mesh_indices = std::vector<std::uint32_t>{0, 1, 2, 3, 2, 1};
  auto const mesh_vertices =
      std::vector<marlon::math::Vec3f>{{-0.5f, -0.5f, 0.0f},
                                       {0.5f, -0.5f, 0.0f},
                                       {-0.5f, 0.5f, 0.0f},
                                       {0.5f, 0.5f, 0.0f}};
  auto const mesh = render_engine->create_mesh(
      {.index_format = rendering::Mesh_index_format::uint32,
       .index_count = 6,
       .index_data = mesh_indices.data(),
       .vertex_format =
           {.position_fetch_info =
                {.format = rendering::Mesh_vertex_position_format::float3,
                 .offset = 0},
            .stride = 12},
       .vertex_count = 4,
       .vertex_data = mesh_vertices.data()});
  auto const surface =
      render_engine->create_surface({.material = material, .mesh = mesh});
  auto const scene = render_engine->create_scene({});
  auto const initial_diff = render_engine->create_scene_diff({.scene = scene});
  auto const surface_node = render_engine->record_scene_node_creation(
      initial_diff, {.translation = math::Vec3f::zero(),
                     .rotation = math::Quatf::identity(),
                     .scale = 1.0f});
  render_engine->record_surface_instance_creation(
      initial_diff, {.surface = surface, .scene_node = surface_node});
  auto const camera = render_engine->record_camera_creation(
      initial_diff, {.near_plane_distance = 0.1f,
                     .far_plane_distance = 1000.0f,
                     .aspect_ratio = 16.0f / 9.0f,
                     .vertical_fov = 1.5f});
  auto const camera_node = render_engine->record_scene_node_creation(
      initial_diff, {.translation = math::Vec3f::zero(),
                     .rotation = math::Quatf::identity(),
                     .scale = 1.0f});
  auto const camera_instance = render_engine->record_camera_instance_creation(
      initial_diff, {.camera = camera, .scene_node = camera_node});
  render_engine->apply_scene_diff(initial_diff);
  render_engine->destroy_scene_diff(initial_diff);
  auto const left_diff = render_engine->create_scene_diff({.scene = scene});
  render_engine->record_scene_node_translation_continuous(
      left_diff, surface_node, {-0.5f, 0.0f, 0.0f});
  render_engine->record_scene_node_rotation_continuous(
      left_diff, surface_node,
      math::Quatf::axis_angle({0.0f, 0.0f, 1.0f}, math::deg_to_rad(-90.0f)));
  render_engine->record_scene_node_scale_continuous(left_diff, surface_node,
                                                    1.0f);
  auto const right_diff = render_engine->create_scene_diff({.scene = scene});
  render_engine->record_scene_node_translation_continuous(
      right_diff, surface_node, {0.5f, 0.0f, 0.0f});
  render_engine->record_scene_node_rotation_continuous(
      right_diff, surface_node,
      math::Quatf::axis_angle({0.0f, 0.0f, 1.0f}, math::deg_to_rad(90.0f)));
  render_engine->record_scene_node_scale_continuous(right_diff, surface_node,
                                                    0.5f);
  auto const diffs = std::vector{left_diff, right_diff};
  auto const time_between_updates = 1.0;
  auto diff_index = 0;
  auto previous_loop_time = glfwGetTime();
  auto time_until_update = 0.0;
  while (!glfwWindowShouldClose(window.get())) {
    glfwPollEvents();
    auto const current_loop_time = glfwGetTime();
    auto const time_passed = current_loop_time - previous_loop_time;
    previous_loop_time = current_loop_time;
    time_until_update -= time_passed;
    while (time_until_update <= 0.0) {
      time_until_update += time_between_updates;
      render_engine->apply_scene_diff(diffs[diff_index]);
      diff_index = 1 - diff_index;
    }
    render_engine->apply_scene_diff(
        diffs[diff_index],
        static_cast<float>(1.0 - time_until_update /
                                     (time_until_update + time_passed)));
    render_engine->render(scene, camera_instance,
                          render_engine->get_default_render_target());
    glfwSwapBuffers(window.get());
  }
  render_engine->destroy_scene_diff(right_diff);
  render_engine->destroy_scene_diff(left_diff);
  render_engine->destroy_surface(surface);
  render_engine->destroy_material(material);
  render_engine->destroy_mesh(mesh);
  render_engine->destroy_scene(scene);
  return 0;
}