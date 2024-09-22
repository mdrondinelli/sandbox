#ifndef MARLON_GAME_BOX_H
#define MARLON_GAME_BOX_H

#include "physics/rigid_body.h"
#include "physics/world.h"
#include "util/pool.h"

namespace marlon::game {
class Box_creation_callback;

class Box_destruction_callback;

class Box_motion_callback;

struct Rigid_box_create_info {
  void *user_pointer;
  math::Vec3f half_extents;
  physics::Material material;
  float mass;
  bool hollow;
  math::Vec3f position;
  math::Quatf orientation;
  math::Vec3f velocity;
  math::Vec3f angular_velocity;
};

struct Static_box_create_info {
  void *user_pointer;
  math::Vec3f half_extents;
  physics::Material material;
  math::Vec3f position;
  math::Quatf orientation;
};

class Box : physics::Rigid_body_motion_callback {
public:
  Box() = default;

  explicit Box(void *user_pointer, physics::World *world, physics::Object body, Box_motion_callback *motion_callback)
      : _user_pointer{user_pointer}, _world{world}, _body{body}, _motion_callback{motion_callback} {}

  void *get_user_pointer() const noexcept {
    return _user_pointer;
  }

  void set_user_pointer(void *user_pointer) noexcept {
    _user_pointer = user_pointer;
  }

  physics::Object get_physics_object() const noexcept {
    return _body;
  }

  math::Vec3f get_half_extents() const noexcept;

  math::Vec3f get_position() const noexcept;

  math::Quatf get_orientation() const noexcept;

  Box_motion_callback *get_motion_callback() const noexcept;

  void set_motion_callback(Box_motion_callback *callback) noexcept;

private:
  void set_rigid_body_motion_callback(physics::Rigid_body_motion_callback *callback) noexcept;

  void on_rigid_body_motion(physics::World const &, physics::Rigid_body) final;

  void *_user_pointer{};
  physics::World *_world{};
  physics::Object _body{};
  Box_motion_callback *_motion_callback{};
};

class Box_creation_callback {
public:
  virtual ~Box_creation_callback() = default;

  virtual void on_box_created(Box *box) = 0;
};

class Box_destruction_callback {
public:
  virtual ~Box_destruction_callback() = default;

  virtual void on_box_destroyed(Box *box) = 0;
};

class Box_motion_callback {
public:
  virtual ~Box_motion_callback() = default;

  virtual void on_box_moved(Box *box) = 0;
};

struct Box_manager_create_info {
  physics::World *world;
  util::Size max_boxes;
};

class Box_manager {
public:
  Box_manager() = default;

  explicit Box_manager(Box_manager_create_info const &create_info);

  Box *create_box(Rigid_box_create_info const &create_info);

  Box *create_box(Static_box_create_info const &create_info);

  void destroy_box(Box *box) noexcept;

  void set_box_creation_callback(Box_creation_callback *callback) noexcept;

  void set_box_destruction_callback(Box_destruction_callback *callback) noexcept;

private:
  util::Block _memory{};
  util::Pool<Box> _boxes{};
  Box_creation_callback *_box_creation_callback{};
  Box_destruction_callback *_box_destruction_callback{};
  physics::World *_world{};
};
} // namespace marlon::game

#endif
