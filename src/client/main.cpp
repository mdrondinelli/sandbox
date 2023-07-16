#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "glfw/instance.h"
#include "glfw/window.h"
#include "rendering/gl/render_engine.h"

static_assert(sizeof(marlon::math::Vec3f) == 12);

int main() {
  marlon::glfw::Instance glfw;
  // glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  const auto window = marlon::glfw::make_unique_window(800, 600, "title");
  glfwMakeContextCurrent(window.get());
  const auto loadproc = [](const char *procname) -> void * {
    return glfwGetProcAddress(procname);
  };
  if (!gladLoadGLLoader(loadproc)) {
    std::cerr << "Failed to initialize OpenGL" << std::endl;
    return -1;
  }
  glfwSwapInterval(1);
  auto const render_engine = new marlon::rendering::Gl_render_engine{};
  auto const material =
      render_engine->create_material({.albedo = {1.0f, 1.0f, 1.0f}});
  auto const mesh_indices = std::vector<std::uint32_t>{0, 1, 2};
  auto const mesh_vertices = std::vector<marlon::math::Vec3f>{
      {-0.5f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}};
  auto const mesh = render_engine->create_mesh(
      {.index_format = marlon::rendering::Mesh_index_format::uint32,
       .index_count = 3,
       .index_data = mesh_indices.data(),
       .vertex_format =
           {.position_fetch_info =
                {.format =
                     marlon::rendering::Mesh_vertex_position_format::float3,
                 .offset = 0},
            .stride = 12},
       .vertex_count = 3,
       .vertex_data = mesh_vertices.data()});
  auto const surface =
      render_engine->create_surface({.material = material, .mesh = mesh});
  auto const scene = render_engine->create_scene({});
  auto const scene_diff = render_engine->create_scene_diff({.scene = scene});
  auto const surface_node = render_engine->record_scene_node_creation(
      scene_diff, {.translation = marlon::math::Vec3f::zero(),
                   .rotation = marlon::math::Quatf::identity(),
                   .scale = 1.0f});
  render_engine->record_surface_instance_creation(
      scene_diff, {.surface = surface, .scene_node = surface_node});
  auto const camera = render_engine->record_camera_creation(
      scene_diff, {.near_plane_distance = 0.1f,
                   .far_plane_distance = 1000.0f,
                   .aspect_ratio = 16.0f / 9.0f,
                   .vertical_fov = 1.5f});
  auto const camera_node = render_engine->record_scene_node_creation(
      scene_diff, {.translation = marlon::math::Vec3f::zero(),
                   .rotation = marlon::math::Quatf::identity(),
                   .scale = 1.0f});
  auto const camera_instance = render_engine->record_camera_instance_creation(
      scene_diff, {.camera = camera, .scene_node = camera_node});
  render_engine->apply_scene_diff(scene_diff);
  render_engine->destroy_scene_diff(scene_diff);
  auto const render_stream = render_engine->create_render_stream(
      {.source_scene = scene,
       .source_camera_instance = camera_instance,
       .destination = render_engine->get_default_render_destination()});
  while (!glfwWindowShouldClose(window.get())) {
    glfwPollEvents();
    render_engine->render(render_stream);
    glfwSwapBuffers(window.get());
  }
  render_engine->destroy_render_stream(render_stream);
  render_engine->destroy_surface(surface);
  render_engine->destroy_material(material);
  render_engine->destroy_mesh(mesh);
  render_engine->destroy_scene(scene);
  delete render_engine;
  return 0;
}