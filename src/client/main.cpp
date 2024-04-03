#include <fstream>
#include <iostream>
#include <stdexcept>

#include "dynamic_prop.h"
#include "static_prop.h"

#include "../engine/app.h"
#include "../graphics/graphics.h"

using namespace marlon;
using enum engine::Key;
using enum engine::Mouse_button;

struct Resources {
  graphics::Unique_texture_ptr brick_base_color_texture;
  graphics::Unique_texture_ptr striped_cotton_base_color_texture;
  graphics::Unique_surface_material_ptr brick_material;
  graphics::Unique_surface_material_ptr striped_cotton_material;
  graphics::Unique_surface_material_ptr red_material;
  graphics::Unique_surface_material_ptr blue_material;
  graphics::Unique_surface_material_ptr particle_material;
  graphics::Unique_surface_mesh_ptr cube_mesh;
  graphics::Unique_surface_mesh_ptr low_quality_sphere_mesh;
  graphics::Unique_surface_mesh_ptr high_quality_sphere_mesh;
};

graphics::Unique_texture_ptr create_texture(graphics::Graphics *graphics,
                                            const char *path);

graphics::Unique_surface_mesh_ptr
create_cuboid_mesh(graphics::Graphics *graphics);

graphics::Unique_surface_mesh_ptr
create_icosphere_mesh(graphics::Graphics *graphics, int subdivisions = 0);

Resources create_resources(graphics::Graphics *graphics) {
  Resources retval;
  retval.brick_base_color_texture = create_texture(
      graphics, "C:/Users/mdron/Sandbox/res/BrickWall29_4K_BaseColor.ktx");
  retval.striped_cotton_base_color_texture = create_texture(
      graphics, "C:/Users/mdron/Sandbox/res/StripedCotton01_2K_BaseColor.ktx");
  retval.brick_material = graphics->create_surface_material_unique(
      {.base_color_texture = retval.brick_base_color_texture.get()});
  retval.striped_cotton_material = graphics->create_surface_material_unique(
      {.base_color_texture = retval.striped_cotton_base_color_texture.get()});
  retval.red_material = graphics->create_surface_material_unique(
      {.base_color_tint = {0.1f, 0.0f, 0.0f}});
  retval.blue_material = graphics->create_surface_material_unique(
      {.base_color_tint = {0.00f, 0.005f, 0.02f}});
  retval.particle_material = graphics->create_surface_material_unique(
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

graphics::Unique_surface_mesh_ptr
create_cuboid_mesh(graphics::Graphics *graphics) {
  std::vector<graphics::Surface_vertex> const vertices{
      // -x face
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
  std::vector<std::uint16_t> const indices{
      0,  1,  2,  3,  2,  1,  4,  5,  6,  7,  6,  5,  8,  9,  10, 11, 10, 9,
      12, 13, 14, 15, 14, 13, 16, 17, 18, 19, 18, 17, 20, 21, 22, 23, 22, 21};
  return graphics->create_surface_mesh_unique(
      {.indices = indices, .vertices = vertices});
}

graphics::Unique_surface_mesh_ptr
create_icosphere_mesh(graphics::Graphics *graphics, int subdivisions) {
  auto const phi = std::numbers::phi_v<float>;
  std::vector<graphics::Surface_vertex> vertices{
      {normalize(math::Vec3f{phi, 1.0f, 0.0f}), {}},
      {normalize(math::Vec3f{phi, -1.0f, 0.0f}), {}},
      {normalize(math::Vec3f{-phi, -1.0f, 0.0f}), {}},
      {normalize(math::Vec3f{-phi, 1.0f, 0.0f}), {}},
      {normalize(math::Vec3f{1.0f, 0.0f, phi}), {}},
      {normalize(math::Vec3f{-1.0f, 0.0f, phi}), {}},
      {normalize(math::Vec3f{-1.0f, 0.0f, -phi}), {}},
      {normalize(math::Vec3f{1.0f, 0.0f, -phi}), {}},
      {normalize(math::Vec3f{0.0f, phi, 1.0f}), {}},
      {normalize(math::Vec3f{0.0f, phi, -1.0f}), {}},
      {normalize(math::Vec3f{0.0f, -phi, -1.0f}), {}},
      {normalize(math::Vec3f{0.0f, -phi, 1.0f}), {}}};
  std::vector<std::uint16_t> indices{
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
      auto const new_vertex_0 = graphics::Surface_vertex{
          normalize(0.5f * (vertex_0.position + vertex_1.position)), {}};
      auto const new_vertex_1 = graphics::Surface_vertex{
          normalize(0.5f * (vertex_1.position + vertex_2.position)), {}};
      auto const new_vertex_2 = graphics::Surface_vertex{
          normalize(0.5f * (vertex_2.position + vertex_0.position)), {}};
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
  return graphics->create_surface_mesh_unique({
      .indices = indices,
      .vertices = vertices,
  });
}

class Client : public engine::App {
public:
  Client()
      : App{{
            .world_create_info =
                {
                    .gravitational_acceleration = {0.0f, -9.8f, 0.0f},
                },
            .world_simulate_info =
                {
                    .substep_count = 24,
                },
            .window_extents = {1600, 900},
        }} {}

  void pre_loop() final {
    auto const world = get_world();
    auto const graphics = get_graphics();
    auto const scene = get_scene();
    auto const camera = get_camera();
    _resources = create_resources(graphics);
    _red_ball_manager = std::make_unique<client::Static_prop_manager>(
        client::Static_prop_manager_create_info{
            .scene = scene,
            .surface_mesh = _resources.high_quality_sphere_mesh.get(),
            .surface_material = _resources.red_material.get(),
            .surface_pretransform = math::Mat3x4f{{0.5f, 0.0f, 0.0f, 0.0f},
                                                  {0.0f, 0.5f, 0.0f, 0.0f},
                                                  {0.0f, 0.0f, 0.5f, 0.0f}},
            .space = world,
            .body_shape = physics::Ball{0.5f},
            .body_material =
                {
                    .static_friction_coefficient = 0.2f,
                    .dynamic_friction_coefficient = 0.1f,
                    .restitution_coefficient = 0.3f,
                },
        });
    physics::Box cotton_box_shape{{0.3f, 0.3f, 0.3f}};
    _cotton_box_manager = std::make_unique<client::Dynamic_prop_manager>(
        client::Dynamic_prop_manager_create_info{
            .scene = scene,
            .surface_mesh = _resources.cube_mesh.get(),
            .surface_material = _resources.striped_cotton_material.get(),
            .surface_pretransform =
                math::Mat3x4f{
                    {cotton_box_shape.half_extents[0], 0.0f, 0.0f, 0.0f},
                    {0.0f, cotton_box_shape.half_extents[1], 0.0f, 0.0f},
                    {0.0f, 0.0f, cotton_box_shape.half_extents[2], 0.0f}},
            .space = world,
            .body_mass = 80.0f,
            .body_inertia_tensor =
                80.0f * physics::solid_inertia_tensor(cotton_box_shape),
            .body_shape = cotton_box_shape,
            .body_material =
                {
                    .static_friction_coefficient = 0.3f,
                    .dynamic_friction_coefficient = 0.2f,
                    .restitution_coefficient = 0.2f,
                },
        });
    _red_ball_manager->create({.position = {-1.5f, 0.5f, -1.5f}});
    _red_ball_manager->create({.position = {1.5f, 0.5f, 1.5f}});
    world->create_static_body({
        .shape = physics::Box{{100.0f, 0.5f, 100.0f}},
        .material =
            {
                .static_friction_coefficient = 0.4f,
                .dynamic_friction_coefficient = 0.3f,
                .restitution_coefficient = 0.1f,
            },
        .position = {0.0f, -0.5f, 0.0f},
    });
    _ground_surface = scene->create_surface_unique(
        {.mesh = _resources.cube_mesh.get(),
         .material = _resources.blue_material.get(),
         .transform = math::Mat3x4f{{100.0f, 0.0f, 0.0f, 0.0f},
                                    {0.0f, 0.5f, 0.0f, -0.5f},
                                    {0.0f, 0.0f, 100.0f, 0.0f}}});
    camera->set_position({-10.0f, 3.5f, 10.0f});
    camera->set_zoom(math::Vec2f{9.0f / 16.0f, 1.0f} * 2.0f);
    srand(25);
  }

  void post_input() final {
    auto const window = get_window();
    if (window->should_close()) {
      stop_looping();
      return;
    }
    auto const camera = get_camera();
    if (window->is_mouse_button_pressed(mb_right)) {
      window->set_cursor_mode(engine::Cursor_mode::disabled);
      auto const rotation_matrix =
          math::Mat3x3f::rotation(camera->get_orientation());
      auto const right_vector = column(rotation_matrix, 0);
      auto const forward_vector = -column(rotation_matrix, 2);
      auto movement = math::Vec3f::zero();
      if (window->is_key_pressed(k_w)) {
        movement += forward_vector;
      }
      if (window->is_key_pressed(k_a)) {
        movement -= right_vector;
      }
      if (window->is_key_pressed(k_s)) {
        movement -= forward_vector;
      }
      if (window->is_key_pressed(k_d)) {
        movement += right_vector;
      }
      if (window->is_key_pressed(k_e)) {
        movement += math::Vec3f::y_axis();
      }
      if (window->is_key_pressed(k_q)) {
        movement -= math::Vec3f::y_axis();
      }
      if (movement != math::Vec3f::zero()) {
        movement = 8.0f * normalize(movement);
      }
      auto const dt = static_cast<float>(get_delta_time());
      camera->set_position(camera->get_position() + movement * dt);
      _camera_yaw -= window->get_delta_cursor_position().x * 0.002f;
      _camera_pitch -= window->get_delta_cursor_position().y * 0.002f;
      _camera_pitch = std::clamp(_camera_pitch,
                                 -0.5f * std::numbers::pi_v<float>,
                                 0.5f * std::numbers::pi_v<float>);
    } else {
      window->set_cursor_mode(engine::Cursor_mode::normal);
    }
    camera->set_orientation(
        math::Quatf::axis_angle(math::Vec3f::y_axis(), _camera_yaw) *
        math::Quatf::axis_angle(math::Vec3f::x_axis(), _camera_pitch));
  }

  void post_physics() final {
    _box_spawn_timer += 1.0f / 64.0f;
    if (_box_spawn_timer > 0.0f) {
      _box_spawn_timer -= 0.1f;
      if (_box_spawn_height > 8.0f) {
        _box_spawn_height = 2.0f;
        _box_spawn_offset_x = 40 * (rand() / (float)RAND_MAX) - 20;
        _box_spawn_offset_z = 40 * (rand() / (float)RAND_MAX) - 20;
      } else {
        _box_spawn_height += 0.6f;
      }
      ++_box_count;
      if (_box_count == 768) {
        _box_spawn_timer = -1000000.0f;
      }
      _cotton_box_manager->create({
          .position = math::Vec3f{_box_spawn_offset_x,
                                  _box_spawn_height,
                                  _box_spawn_offset_z},
          .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
          .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f},
                                                 math::deg_to_rad(90.0f)),
          .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
      });
    }
  }

private:
  Resources _resources;
  std::unique_ptr<client::Static_prop_manager> _red_ball_manager;
  std::unique_ptr<client::Dynamic_prop_manager> _cotton_box_manager;
  graphics::Unique_surface_ptr _ground_surface;
  float _camera_yaw{math::deg_to_rad(-45.0f)};
  float _camera_pitch{0.0f};
  int _box_count{0};
  double _box_spawn_timer{0.0};
  float _box_spawn_height{0.5f};
  float _box_spawn_offset_x{0.0f};
  float _box_spawn_offset_z{0.0f};
};

int main() { return Client{}.run(); }