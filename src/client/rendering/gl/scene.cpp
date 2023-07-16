#include "scene.h"

namespace marlon {
namespace rendering {
Gl_scene::Impl::Impl(Scene_create_info const &) noexcept {}

Gl_scene::Impl::~Impl() {
  for (auto const surface_instance : _surface_instances) {
    delete surface_instance;
  }
  for (auto const camera_instance : _camera_instances) {
    delete camera_instance;
  }
  for (auto const camera : _cameras) {
    delete camera;
  }
  for (auto const scene_node : _scene_nodes) {
    delete scene_node;
  }
}

void Gl_scene::Impl::acquire_scene_node(
    std::unique_ptr<Gl_scene_node> scene_node) {
  _scene_nodes.emplace(scene_node.get());
  scene_node.release();
}

bool Gl_scene::Impl::release_scene_node(Gl_scene_node *scene_node) {
  return _scene_nodes.erase(scene_node) != 0;
}

void Gl_scene::Impl::acquire_camera(std::unique_ptr<Gl_camera> camera) {
  _cameras.emplace(camera.get());
  camera.release();
}

bool Gl_scene::Impl::release_camera(Gl_camera *camera) {
  return _cameras.erase(camera) != 0;
}

void Gl_scene::Impl::acquire_camera_instance(
    std::unique_ptr<Gl_camera_instance> camera_instance) {
  _camera_instances.emplace(camera_instance.get());
  camera_instance.release();
}

bool Gl_scene::Impl::release_camera_instance(
    Gl_camera_instance *camera_instance) {
  return _camera_instances.erase(camera_instance) != 0;
}

void Gl_scene::Impl::acquire_surface_instance(
    std::unique_ptr<Gl_surface_instance> surface_instance) {
  _surface_instances.emplace(surface_instance.get());
  surface_instance.release();
}

bool Gl_scene::Impl::release_surface_instance(
    Gl_surface_instance *surface_instance) {
  return _surface_instances.erase(surface_instance) != 0;
}

void Gl_scene::Impl::draw_surface_instances() {
  for (auto const surface_instance : _surface_instances) {
    auto const &mesh_impl =
        surface_instance->_impl.get_surface()->_impl.get_mesh()->_impl;
    mesh_impl.bind_vertex_array();
    mesh_impl.draw();
  }
}

Gl_scene::Gl_scene(Scene_create_info const &create_info) noexcept
    : _impl{create_info} {}
} // namespace rendering
} // namespace marlon