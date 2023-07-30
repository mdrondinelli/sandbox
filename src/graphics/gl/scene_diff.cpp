#include "scene_diff.h"

namespace marlon {
namespace graphics {
Gl_scene_diff::Impl::Impl(Scene_diff_create_info const &create_info) noexcept
    : _scene{static_cast<Gl_scene *>(create_info.scene)} {}

Gl_scene_node *Gl_scene_diff::Impl::record_scene_node_creation(
    Scene_node_create_info const &create_info) {
  return _created_scene_nodes
      .emplace_back(std::make_unique<Gl_scene_node>(create_info))
      .get();
}

void Gl_scene_diff::Impl::record_scene_node_destruction(
    Gl_scene_node *scene_node) {
  _destroyed_scene_nodes.emplace_back(scene_node);
}

void Gl_scene_diff::Impl::record_scene_node_translation_continuous(
    Gl_scene_node *scene_node, math::Vec3f const &value) {
  _continuous_scene_node_translations.emplace_back(scene_node, value);
}

void Gl_scene_diff::Impl::record_scene_node_translation_discontinuous(
    Gl_scene_node *scene_node, math::Vec3f const &value) {
  _discontinuous_scene_node_translations.emplace_back(scene_node, value);
}

void Gl_scene_diff::Impl::record_scene_node_rotation_continuous(
    Gl_scene_node *scene_node, math::Quatf const &value) {
  _continuous_scene_node_rotations.emplace_back(scene_node, value);
}

void Gl_scene_diff::Impl::record_scene_node_rotation_discontinuous(
    Gl_scene_node *scene_node, math::Quatf const &value) {
  _discontinuous_scene_node_rotations.emplace_back(scene_node, value);
}

void Gl_scene_diff::Impl::record_scene_node_scale_continuous(
    Gl_scene_node *scene_node, float value) {
  _continuous_scene_node_scales.emplace_back(scene_node, value);
}

void Gl_scene_diff::Impl::record_scene_node_scale_discontinuous(
    Gl_scene_node *scene_node, float value) {
  _discontinuous_scene_node_scales.emplace_back(scene_node, value);
}

Gl_camera *Gl_scene_diff::Impl::record_camera_creation(
    Camera_create_info const &create_info) {
  return _created_cameras.emplace_back(std::make_unique<Gl_camera>(create_info))
      .get();
}

void Gl_scene_diff::Impl::record_camera_destruction(Gl_camera *camera) {
  _destroyed_cameras.emplace_back(camera);
}

Gl_camera_instance *Gl_scene_diff::Impl::record_camera_instance_creation(
    Camera_instance_create_info const &create_info) {
  return _created_camera_instances
      .emplace_back(std::make_unique<Gl_camera_instance>(create_info))
      .get();
}

void Gl_scene_diff::Impl::record_camera_instance_destruction(
    Gl_camera_instance *camera_instance) {
  _destroyed_camera_instances.emplace_back(camera_instance);
}

Gl_surface_instance *Gl_scene_diff::Impl::record_surface_instance_creation(
    Surface_instance_create_info const &create_info) {
  return _created_surface_instances
      .emplace_back(std::make_unique<Gl_surface_instance>(create_info))
      .get();
}

void Gl_scene_diff::Impl::record_surface_instance_destruction(
    Gl_surface_instance *surface_instance) {
  _destroyed_surface_instances.emplace_back(surface_instance);
}

void Gl_scene_diff::Impl::apply() {
  apply_continuous();
  apply_discontinuous();
  apply_created_cameras();
  apply_created_scene_nodes();
  apply_created_camera_instances();
  apply_created_surface_instances();
  apply_destroyed_surface_instances();
  apply_destroyed_camera_instances();
  apply_destroyed_scene_nodes();
  apply_destroyed_cameras();
}

void Gl_scene_diff::Impl::apply(float factor) {
  for (auto const &pair : _continuous_scene_node_translations) {
    pair.first->_impl.blend_translation(pair.second, factor);
  }
  for (auto const &pair : _continuous_scene_node_rotations) {
    pair.first->_impl.blend_rotation(pair.second, factor);
  }
  for (auto const &pair : _continuous_scene_node_scales) {
    pair.first->_impl.blend_scale(pair.second, factor);
  }
}

void Gl_scene_diff::Impl::apply_continuous() {
  for (auto const &pair : _continuous_scene_node_translations) {
    pair.first->_impl.set_translation(pair.second);
  }
  for (auto const &pair : _continuous_scene_node_rotations) {
    pair.first->_impl.set_rotation(pair.second);
  }
  for (auto const &pair : _continuous_scene_node_scales) {
    pair.first->_impl.set_scale(pair.second);
  }
  _continuous_scene_node_translations.clear();
  _continuous_scene_node_rotations.clear();
  _continuous_scene_node_scales.clear();
}

void Gl_scene_diff::Impl::apply_discontinuous() {
  for (auto const &pair : _discontinuous_scene_node_translations) {
    pair.first->_impl.set_translation(pair.second);
  }
  for (auto const &pair : _discontinuous_scene_node_rotations) {
    pair.first->_impl.set_rotation(pair.second);
  }
  for (auto const &pair : _discontinuous_scene_node_scales) {
    pair.first->_impl.set_scale(pair.second);
  }
  _discontinuous_scene_node_translations.clear();
  _discontinuous_scene_node_rotations.clear();
  _discontinuous_scene_node_scales.clear();
}

void Gl_scene_diff::Impl::apply_created_cameras() {
  for (auto &camera : _created_cameras) {
    _scene->_impl.acquire_camera(std::move(camera));
  }
  _created_cameras.clear();
}

void Gl_scene_diff::Impl::apply_created_scene_nodes() {
  for (auto &scene_node : _created_scene_nodes) {
    _scene->_impl.acquire_scene_node(std::move(scene_node));
  }
  _created_scene_nodes.clear();
}

void Gl_scene_diff::Impl::apply_created_camera_instances() {
  for (auto &camera_instance : _created_camera_instances) {
    _scene->_impl.acquire_camera_instance(std::move(camera_instance));
  }
  _created_camera_instances.clear();
}

void Gl_scene_diff::Impl::apply_created_surface_instances() {
  for (auto &surface_instance : _created_surface_instances) {
    _scene->_impl.acquire_surface_instance(std::move(surface_instance));
  }
  _created_surface_instances.clear();
}

void Gl_scene_diff::Impl::apply_destroyed_surface_instances() {
  for (auto const surface_instance : _destroyed_surface_instances) {
    if (_scene->_impl.release_surface_instance(surface_instance)) {
      delete surface_instance;
    }
  }
  _destroyed_surface_instances.clear();
}

void Gl_scene_diff::Impl::apply_destroyed_camera_instances() {
  for (auto const camera_instance : _destroyed_camera_instances) {
    _scene->_impl.release_camera_instance(camera_instance);
  }
  _destroyed_camera_instances.clear();
}

void Gl_scene_diff::Impl::apply_destroyed_scene_nodes() {
  for (auto const scene_node : _destroyed_scene_nodes) {
    if (_scene->_impl.release_scene_node(scene_node)) {
      delete scene_node;
    }
  }
  _destroyed_scene_nodes.clear();
}

void Gl_scene_diff::Impl::apply_destroyed_cameras() {
  for (auto const camera : _destroyed_cameras) {
    if (_scene->_impl.release_camera(camera)) {
      delete camera;
    }
  }
  _destroyed_cameras.clear();
}

Gl_scene_diff::Gl_scene_diff(Scene_diff_create_info const &create_info) noexcept
    : _impl{create_info} {}

Gl_scene_node *Gl_scene_diff::record_scene_node_creation(
    Scene_node_create_info const &create_info) {
  return _impl.record_scene_node_creation(create_info);
}

void Gl_scene_diff::record_scene_node_destruction(Scene_node *scene_node) {
  _impl.record_scene_node_destruction(static_cast<Gl_scene_node *>(scene_node));
}

void Gl_scene_diff::record_scene_node_translation_continuous(
    Scene_node *scene_node, math::Vec3f const &value) {
  _impl.record_scene_node_translation_continuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_scene_diff::record_scene_node_translation_discontinuous(
    Scene_node *scene_node, math::Vec3f const &value) {
  _impl.record_scene_node_translation_discontinuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_scene_diff::record_scene_node_rotation_continuous(
    Scene_node *scene_node, math::Quatf const &value) {
  _impl.record_scene_node_rotation_continuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_scene_diff::record_scene_node_rotation_discontinuous(
    Scene_node *scene_node, math::Quatf const &value) {
  _impl.record_scene_node_rotation_discontinuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_scene_diff::record_scene_node_scale_continuous(Scene_node *scene_node,
                                                       float value) {
  _impl.record_scene_node_scale_continuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

void Gl_scene_diff::record_scene_node_scale_discontinuous(
    Scene_node *scene_node, float value) {
  _impl.record_scene_node_scale_discontinuous(
      static_cast<Gl_scene_node *>(scene_node), value);
}

Gl_camera *
Gl_scene_diff::record_camera_creation(Camera_create_info const &create_info) {
  return _impl.record_camera_creation(create_info);
}

void Gl_scene_diff::record_camera_destruction(Camera *camera) {
  _impl.record_camera_destruction(static_cast<Gl_camera *>(camera));
}

Gl_camera_instance *Gl_scene_diff::record_camera_instance_creation(
    Camera_instance_create_info const &create_info) {
  return _impl.record_camera_instance_creation(create_info);
}

void Gl_scene_diff::record_camera_instance_destruction(
    Camera_instance *camera_instance) {
  _impl.record_camera_instance_destruction(
      static_cast<Gl_camera_instance *>(camera_instance));
}

Gl_surface_instance *Gl_scene_diff::record_surface_instance_creation(
    Surface_instance_create_info const &create_info) {
  return _impl.record_surface_instance_creation(create_info);
}

void Gl_scene_diff::record_surface_instance_destruction(
    Surface_instance *surface_instance) {
  _impl.record_surface_instance_destruction(
      static_cast<Gl_surface_instance *>(surface_instance));
}
} // namespace graphics
} // namespace marlon