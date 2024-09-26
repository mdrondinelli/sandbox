#include "box.h"
#include "math/vec.h"
#include "physics/world.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon::game {
TEST_CASE("Box_manager works") {
  struct Callback : Box_creation_callback, Box_destruction_callback {
    Callback(bool *creation_flag, bool *destruction_flag)
        : creation_flag{creation_flag}, destruction_flag{destruction_flag} {}

    void on_box_created(Box *) {
      *creation_flag = true;
    }

    void on_box_destroyed(Box *) {
      *destruction_flag = true;
    }

    bool *creation_flag;
    bool *destruction_flag;
  };
  auto const rigid_box_create_info = Rigid_box_create_info{
      .user_pointer = nullptr,
      .half_extents = math::Vec3f::all(0.5f),
      .material = physics::Material{0.0f, 0.0f, 0.0f},
      .mass = 1.0f,
      .hollow = false,
      .position = math::Vec3f::zero(),
      .orientation = math::Quatf::identity(),
      .velocity = math::Vec3f::zero(),
      .angular_velocity = math::Vec3f::zero(),
  };
  auto const static_box_create_info = Static_box_create_info{
      .user_pointer = nullptr,
      .half_extents = math::Vec3f::all(0.5f),
      .material = physics::Material{0.0f, 0.0f, 0.0f},
      .position = math::Vec3f::zero(),
      .orientation = math::Quatf::identity(),
  };
  auto world = physics::World{{}};
  auto boxes = Box_manager{{.world = &world, .max_boxes = 100}};
  auto creation_callback_called = false;
  auto destruction_callback_called = false;
  auto callback =
      Callback{&creation_callback_called, &destruction_callback_called};
  boxes.set_box_creation_callback(&callback);
  boxes.set_box_destruction_callback(&callback);
  REQUIRE(creation_callback_called == false);
  REQUIRE(destruction_callback_called == false);
  auto box = boxes.create_box(rigid_box_create_info);
  REQUIRE(creation_callback_called == true);
  REQUIRE(destruction_callback_called == false);
  creation_callback_called = false;
  destruction_callback_called = false;
  boxes.destroy_box(box);
  box = nullptr;
  REQUIRE(creation_callback_called == false);
  REQUIRE(destruction_callback_called == true);
  creation_callback_called = false;
  destruction_callback_called = false;
  box = boxes.create_box(static_box_create_info);
  REQUIRE(creation_callback_called == true);
  REQUIRE(destruction_callback_called == false);
  creation_callback_called = false;
  destruction_callback_called = false;
  boxes.destroy_box(box);
  box = nullptr;
  REQUIRE(creation_callback_called == false);
  REQUIRE(destruction_callback_called == true);
  creation_callback_called = false;
  destruction_callback_called = false;
  boxes.set_box_creation_callback(nullptr);
  boxes.set_box_destruction_callback(nullptr);
  boxes.destroy_box(boxes.create_box(rigid_box_create_info));
  boxes.destroy_box(boxes.create_box(static_box_create_info));
  REQUIRE(creation_callback_called == false);
  REQUIRE(destruction_callback_called == false);
}
} // namespace marlon::game
