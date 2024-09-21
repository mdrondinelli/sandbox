#include "box.h"
#include "util/memory.h"

namespace marlon::game {
math::Vec3f Box::get_half_extents() const noexcept {
  return std::visit(
      [&](auto const body) -> math::Vec3f {
        using T = std::decay_t<decltype(body)>;
        if constexpr (std::is_same_v<T, physics::Rigid_body> || std::is_same_v<T, physics::Static_body>) {
          auto const &shape = _world->data(body)->shape();
          auto const &box = std::get<physics::Box>(shape.get());
          return box.half_extents;
        } else {
          math::unreachable();
        }
      },
      _body.specific());
}

math::Vec3f Box::get_position() const noexcept {
  return std::visit(
      [&](auto const body) -> math::Vec3f {
        using T = std::decay_t<decltype(body)>;
        if constexpr (std::is_same_v<T, physics::Rigid_body> || std::is_same_v<T, physics::Static_body>) {
          return _world->data(body)->position();
        } else {
          math::unreachable();
        }
      },
      _body.specific());
}

math::Quatf Box::get_orientation() const noexcept {
  return std::visit(
      [&](auto const body) -> math::Quatf {
        using T = std::decay_t<decltype(body)>;
        if constexpr (std::is_same_v<T, physics::Rigid_body> || std::is_same_v<T, physics::Static_body>) {
          return _world->data(body)->orientation();
        } else {
          math::unreachable();
        }
      },
      _body.specific());
}

Box_motion_callback *Box::get_motion_callback() const noexcept {
  return _motion_callback;
}

void Box::set_motion_callback(Box_motion_callback *callback) noexcept {
  if (_body.type() == physics::Object_type::rigid_body) {
    if (!_motion_callback && callback) {
      set_rigid_body_motion_callback(this);
    } else if (_motion_callback && !callback) {
      set_rigid_body_motion_callback(nullptr);
    }
  }
  _motion_callback = callback;
}

void Box::set_rigid_body_motion_callback(physics::Rigid_body_motion_callback *callback) noexcept {
  _world->data(std::get<physics::Rigid_body>(_body.specific()))->motion_callback(callback);
}

void Box::on_rigid_body_motion(physics::World const &, physics::Rigid_body) {
  _motion_callback->on_box_moved(this);
}

Box_manager::Box_manager(Box_manager_create_info const &create_info)
    : _world{create_info.world} {
  _memory = util::assign_merged(util::System_allocator{}, std::tie(_boxes), std::tuple{create_info.max_boxes});
}

Box *Box_manager::create_box(Rigid_box_create_info const &create_info) {
  auto const shape = physics::Box{.half_extents = create_info.half_extents};
  auto const body = _world->create_rigid_body({
      .shape = shape,
      .mass = create_info.mass,
      .inertia_tensor = create_info.mass * (create_info.hollow ? physics::surface_inertia_tensor(shape)
                                                               : physics::solid_inertia_tensor(shape)),
      .material = create_info.material,
      .position = create_info.position,
      .velocity = create_info.velocity,
      .orientation = create_info.orientation,
      .angular_velocity = create_info.angular_velocity,
  });
  auto box = static_cast<Box *>(nullptr);
  try {
    box = _boxes.emplace(create_info.user_pointer, _world, body, nullptr);
  } catch (...) {
    _world->destroy_object(body);
    throw;
  }
  if (_box_creation_callback) {
    _box_creation_callback->on_box_created(box);
  }
  return box;
}

Box *Box_manager::create_box(Static_box_create_info const &create_info) {
  auto const shape = physics::Box{.half_extents = create_info.half_extents};
  auto const body = _world->create_static_body({
      .shape = shape,
      .material = create_info.material,
      .position = create_info.position,
      .orientation = create_info.orientation,
  });
  auto box = static_cast<Box *>(nullptr);
  try {
    box = _boxes.emplace(create_info.user_pointer, _world, body, nullptr);
  } catch (...) {
    _world->destroy_object(body);
    throw;
  }
  return box;
}

void Box_manager::destroy_box(Box *box) noexcept {
  _world->destroy_object(box->get_physics_object());
}

void Box_manager::set_box_creation_callback(Box_creation_callback *callback) noexcept {
  _box_creation_callback = callback;
}

void Box_manager::set_box_destruction_callback(Box_destruction_callback *callback) noexcept {
  _box_destruction_callback = callback;
}
} // namespace marlon::game
