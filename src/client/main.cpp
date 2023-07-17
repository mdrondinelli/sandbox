#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "glfw/instance.h"
#include "glfw/window.h"
#include "rendering/gl/render_engine.h"

using namespace marlon;

class Demo {
public:
  Demo()
      : _window{[]() {
          glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
          glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
          glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
          glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
          return glfw::make_unique_window(1024, 768, "title");
        }()},
        _graphics{[this]() {
          glfwMakeContextCurrent(_window.get());
          if (!gladLoadGLLoader([](char const *procname) {
                return static_cast<void *>(glfwGetProcAddress(procname));
              })) {
            throw std::runtime_error{"Failed to initialize OpenGL"};
          }
          return std::make_unique<rendering::Gl_render_engine>();
        }()} {
    glfwSwapInterval(1);
  }

  void run() {
    auto const material =
        _graphics->create_material({.albedo = {1.0f, 0.0f, 0.0f}});
    std::vector<std::uint32_t> const mesh_indices{0, 1, 2, 3, 2, 1};
    std::vector<marlon::math::Vec3f> const mesh_vertices{{-0.5f, -0.5f, 0.0f},
                                                         {0.5f, -0.5f, 0.0f},
                                                         {-0.5f, 0.5f, 0.0f},
                                                         {0.5f, 0.5f, 0.0f}};
    auto const mesh = _graphics->create_mesh(
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
        _graphics->create_surface({.material = material, .mesh = mesh});
    auto const scene = _graphics->create_scene({});
    auto const initial_diff = _graphics->create_scene_diff({.scene = scene});
    auto const surface_node = _graphics->record_scene_node_creation(
        initial_diff, {.translation = math::Vec3f::zero(),
                       .rotation = math::Quatf::identity(),
                       .scale = 2.0f});
    _graphics->record_surface_instance_creation(
        initial_diff, {.surface = surface, .scene_node = surface_node});
    auto const camera = _graphics->record_camera_creation(
        initial_diff, {.near_plane_distance = 0.1f,
                       .far_plane_distance = 1000.0f,
                       .zoom_x = 0.75f,
                       .zoom_y = 1.0f});
    auto const camera_node = _graphics->record_scene_node_creation(
        initial_diff, {.translation = math::Vec3f{0.0f, 0.0f, 10.0f},
                       .rotation = math::Quatf::identity(),
                       .scale = 1.0f});
    auto const camera_instance = _graphics->record_camera_instance_creation(
        initial_diff, {.camera = camera, .scene_node = camera_node});
    _graphics->apply_scene_diff(initial_diff);
    _graphics->destroy_scene_diff(initial_diff);
    auto const left_diff = _graphics->create_scene_diff({.scene = scene});
    _graphics->record_scene_node_translation_continuous(left_diff, surface_node,
                                                        {-5.0f, 0.0f, 0.0f});
    _graphics->record_scene_node_rotation_continuous(
        left_diff, surface_node,
        math::Quatf::axis_angle({0.0f, 1.0f, 0.0f}, math::deg_to_rad(-90.0f)));
    auto const right_diff = _graphics->create_scene_diff({.scene = scene});
    _graphics->record_scene_node_translation_continuous(
        right_diff, surface_node, {5.0f, 0.0f, 0.0f});
    _graphics->record_scene_node_rotation_continuous(
        right_diff, surface_node,
        math::Quatf::axis_angle({0.0f, 1.0f, 0.0f}, math::deg_to_rad(90.0f)));
    auto const diffs = std::vector{left_diff, right_diff};
    auto const time_between_updates = 1.0 / 1.0;
    auto diff_index = 0;
    auto previous_loop_time = glfwGetTime();
    auto time_until_update = 0.0;
    while (!glfwWindowShouldClose(_window.get())) {
      glfwPollEvents();
      auto const current_loop_time = glfwGetTime();
      auto const time_passed = current_loop_time - previous_loop_time;
      previous_loop_time = current_loop_time;
      time_until_update -= time_passed;
      while (time_until_update <= 0.0) {
        time_until_update += time_between_updates;
        _graphics->apply_scene_diff(diffs[diff_index]);
        diff_index = 1 - diff_index;
      }
      _graphics->apply_scene_diff(
          diffs[diff_index],
          static_cast<float>(1.0 - time_until_update /
                                       (time_until_update + time_passed)));
      _graphics->render(scene, camera_instance,
                        _graphics->get_default_render_target());
      glfwSwapBuffers(_window.get());
    }
    _graphics->destroy_scene_diff(right_diff);
    _graphics->destroy_scene_diff(left_diff);
    _graphics->destroy_surface(surface);
    _graphics->destroy_material(material);
    _graphics->destroy_mesh(mesh);
    _graphics->destroy_scene(scene);
  }

private:
  glfw::Instance _glfw;
  glfw::Unique_window_ptr _window;
  std::unique_ptr<rendering::Gl_render_engine> _graphics;
};

int main() {
  Demo demo;
  demo.run();
}