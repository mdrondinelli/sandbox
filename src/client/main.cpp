#include <iostream>

#include "dynamic_prop.h"

#include "../engine/app.h"
#include "../graphics/graphics.h"

using namespace marlon;
using enum engine::Key;
using enum engine::Mouse_button;

struct Resources {
  graphics::Surface_material brick_material;
  graphics::Surface_material striped_cotton_material;
  graphics::Surface_material ground_material;
  graphics::Unique_surface_mesh cube_mesh;
  graphics::Unique_surface_mesh low_quality_sphere_mesh;
  graphics::Unique_surface_mesh high_quality_sphere_mesh;
};

graphics::Unique_surface_mesh create_cube_mesh(graphics::Graphics *graphics);

graphics::Unique_surface_mesh create_icosphere_mesh(graphics::Graphics *graphics, int subdivisions = 0);

Resources create_resources(graphics::Graphics *graphics) {
  Resources retval;
  retval.brick_material = graphics::Surface_material{
      //.base_color_texture = retval.brick_base_color_texture.get(),
  };
  retval.striped_cotton_material = graphics::Surface_material{
      //.base_color_texture = retval.striped_cotton_base_color_texture.get(),
  };
  retval.ground_material = graphics::Surface_material{
      .base_color_tint = graphics::Rgb_spectrum{0.3f},
  };
  retval.cube_mesh = create_cube_mesh(graphics);
  retval.low_quality_sphere_mesh = create_icosphere_mesh(graphics, 1);
  retval.high_quality_sphere_mesh = create_icosphere_mesh(graphics, 3);
  return retval;
}

graphics::Unique_surface_mesh create_cube_mesh(graphics::Graphics *graphics) {
  std::vector<graphics::Surface_vertex> const vertices{// -x face
                                                       {{-1.0f, -1.0f, -1.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
                                                       {{-1.0f, -1.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}},
                                                       {{-1.0f, 1.0f, -1.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
                                                       {{-1.0f, 1.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
                                                       // +x face
                                                       {{1.0f, -1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
                                                       {{1.0f, -1.0f, -1.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}},
                                                       {{1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
                                                       {{1.0f, 1.0f, -1.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
                                                       // -y face
                                                       {{-1.0f, -1.0f, -1.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f}},
                                                       {{1.0f, -1.0f, -1.0f}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f}},
                                                       {{-1.0f, -1.0f, 1.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f}},
                                                       {{1.0f, -1.0f, 1.0f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f}},
                                                       // +y face
                                                       {{-1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
                                                       {{1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
                                                       {{-1.0f, 1.0f, -1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
                                                       {{1.0f, 1.0f, -1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
                                                       // -z face
                                                       {{1.0f, -1.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f}},
                                                       {{-1.0f, -1.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {1.0f, 0.0f}},
                                                       {{1.0f, 1.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f}},
                                                       {{-1.0f, 1.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {1.0f, 1.0f}},
                                                       // +z face
                                                       {{-1.0f, -1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
                                                       {{1.0f, -1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
                                                       {{-1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},
                                                       {{1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}}};
  std::vector<std::uint16_t> const indices{0,  1,  2,  3,  2,  1,  4,  5,  6,  7,  6,  5,  8,  9,  10, 11, 10, 9,
                                           12, 13, 14, 15, 14, 13, 16, 17, 18, 19, 18, 17, 20, 21, 22, 23, 22, 21};
  return graphics->create_surface_mesh_unique({.indices = indices, .vertices = vertices});
}

graphics::Unique_surface_mesh create_icosphere_mesh(graphics::Graphics *graphics, int subdivisions) {
  auto const phi = std::numbers::phi_v<float>;
  std::vector<graphics::Surface_vertex> vertices{{normalize(math::Vec3f{phi, 1.0f, 0.0f}), {}, {}},
                                                 {normalize(math::Vec3f{phi, -1.0f, 0.0f}), {}, {}},
                                                 {normalize(math::Vec3f{-phi, -1.0f, 0.0f}), {}, {}},
                                                 {normalize(math::Vec3f{-phi, 1.0f, 0.0f}), {}, {}},
                                                 {normalize(math::Vec3f{1.0f, 0.0f, phi}), {}, {}},
                                                 {normalize(math::Vec3f{-1.0f, 0.0f, phi}), {}, {}},
                                                 {normalize(math::Vec3f{-1.0f, 0.0f, -phi}), {}, {}},
                                                 {normalize(math::Vec3f{1.0f, 0.0f, -phi}), {}, {}},
                                                 {normalize(math::Vec3f{0.0f, phi, 1.0f}), {}, {}},
                                                 {normalize(math::Vec3f{0.0f, phi, -1.0f}), {}, {}},
                                                 {normalize(math::Vec3f{0.0f, -phi, -1.0f}), {}, {}},
                                                 {normalize(math::Vec3f{0.0f, -phi, 1.0f}), {}, {}}};
  std::vector<std::uint16_t> indices{0, 9, 8,  0, 8,  4,  0,  4, 1, 0, 1, 7,  0, 7,  9,  3, 8,  9,  3, 9,
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
      auto const new_vertex_0 =
          graphics::Surface_vertex{normalize(0.5f * (vertex_0.position + vertex_1.position)), {}, {}};
      auto const new_vertex_1 =
          graphics::Surface_vertex{normalize(0.5f * (vertex_1.position + vertex_2.position)), {}, {}};
      auto const new_vertex_2 =
          graphics::Surface_vertex{normalize(0.5f * (vertex_2.position + vertex_0.position)), {}, {}};
      auto const new_vertex_0_index = static_cast<std::uint32_t>(vertices.size());
      auto const new_vertex_1_index = static_cast<std::uint32_t>(vertices.size() + 1);
      auto const new_vertex_2_index = static_cast<std::uint32_t>(vertices.size() + 2);
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
  for (auto &vertex : vertices) {
    vertex.normal = normalize(vertex.position);
  }
  return graphics->create_surface_mesh_unique({
      .indices = indices,
      .vertices = vertices,
  });
}

constexpr float physics_delta_time = 1.0f / 120.0f;
constexpr int physics_substeps = 10;

class Phase {
public:
  virtual void on_start() = 0;

  virtual void post_physics() = 0;

  virtual void on_stop() = 0;

  void start() {
    if (!_running) {
      _running = true;
      on_start();
    }
  }

  void stop() {
    if (_running) {
      on_stop();
      _running = false;
    }
  }

  bool is_running() const noexcept {
    return _running;
  }

private:
  bool _running{false};
};

class Column_phase : public Phase {
public:
  Column_phase() = default;

  explicit Column_phase(client::Dynamic_prop_manager *box_manager,
                        std::optional<client::Dynamic_prop_handle> *selection)
      : _box_manager{box_manager}, _selection{selection} {
    _boxes.reserve(768);
  }

  void on_start() final {
    _box_spawn_timer = 0.0f;
    _box_spawn_x = 0.0f;
    _box_spawn_y = 0.5f;
    _box_spawn_z = 0.0f;
    srand(25);
    std::cout << "Column phase:\n";
  }

  void post_physics() final {
    _box_spawn_timer += physics_delta_time;
    if (_box_spawn_timer > 0.0f) {
      _box_spawn_timer -= 0.01f;
      if (_box_spawn_y > 8.0f) {
        _box_spawn_y = 2.0f;
        _box_spawn_x = 40 * (rand() / (float)RAND_MAX) - 20;
        _box_spawn_z = 40 * (rand() / (float)RAND_MAX) - 20;
      } else {
        _box_spawn_y += 0.6f;
      }
      auto const box = _box_manager->create({
          .position = math::Vec3f{_box_spawn_x, _box_spawn_y, _box_spawn_z},
          .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
          .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f}, math::deg_to_rad(90.0f)),
          .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
      });
      _boxes.emplace_back(box);
      if (_boxes.size() == 565) {
        *_selection = box;
      } else if (_boxes.size() == 768) {
        stop();
      }
    }
  }

  void on_stop() final {
    for (auto const box : _boxes) {
      _box_manager->destroy(box);
    }
    _boxes.clear();
    *_selection = std::nullopt;
  }

private:
  client::Dynamic_prop_manager *_box_manager{};
  std::optional<client::Dynamic_prop_handle> *_selection{};
  util::Allocating_list<client::Dynamic_prop_handle> _boxes{};
  float _box_spawn_timer{};
  float _box_spawn_x{};
  float _box_spawn_y{};
  float _box_spawn_z{};
};

constexpr auto pyramid_layers = 11;

class Pyramid_phase : public Phase {
public:
  Pyramid_phase() = default;

  explicit Pyramid_phase(client::Dynamic_prop_manager *box_manager)
      : _box_manager{box_manager} {}

  void on_start() final {
    _box_spawn_timer = 0.0f;
    _box_spawn_layer = 0;
    _box_spawn_row = 0;
    _box_spawn_col = 0;
    _timer_started = false;
    _timer = 0.0f;
    std::cout << "Pyramid phase:\n";
  }

  void post_physics() final {
    auto constexpr spacing = 0.61f;
    _box_spawn_timer += physics_delta_time;
    if (_timer_started) {
      _timer += physics_delta_time;
      if (_timer > 1.0f) {
        stop();
      }
    } else if (_box_spawn_timer > 0.0f) {
      _box_spawn_timer -= 0.1f;
      auto const size = pyramid_layers - _box_spawn_layer;
      _boxes.emplace_back(_box_manager->create({
          .position = math::Vec3f{spacing * _box_spawn_row - 0.5f * spacing * size,
                                  spacing * _box_spawn_layer + 0.4f,
                                  spacing * _box_spawn_col - 0.5f * spacing * size},
          // .position = math::Vec3f{0.0f, 0.4f + _box_spawn_col * 10.0f, 0.0f},
          .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
          .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f}, math::deg_to_rad(90.0f)),
          .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
      }));
      if (++_box_spawn_col == size) {
        _box_spawn_col = 0;
        if (++_box_spawn_row == size) {
          _box_spawn_row = 0;
          if (++_box_spawn_layer == pyramid_layers) {
            _timer_started = true;
          }
        }
      }
    }
  }

  void on_stop() final {
    for (auto const box : _boxes) {
      _box_manager->destroy(box);
    }
    _boxes.clear();
  }

private:
  client::Dynamic_prop_manager *_box_manager;
  util::Allocating_list<client::Dynamic_prop_handle> _boxes;
  float _box_spawn_timer{};
  int _box_spawn_layer{};
  int _box_spawn_row{};
  int _box_spawn_col{};
  bool _timer_started{};
  float _timer{};
};

class Ring_phase : public Phase {
public:
  Ring_phase() = default;

  explicit Ring_phase(client::Dynamic_prop_manager *box_manager)
      : _box_manager{box_manager}, _boxes{} {
    _boxes.reserve(768);
  }

  void on_start() final {
    _box_spawn_timer = 0.0f;
    _box_spawn_angle = 0.0f;
    std::cout << "Ring phase:\n";
  }

  void post_physics() final {
    _box_spawn_timer += physics_delta_time;
    _box_spawn_angle += physics_delta_time;
    if (_box_spawn_timer > 0.0f) {
      _box_spawn_timer -= 0.1f;
      auto const new_box = _box_manager->create({
          .position = math::Vec3f{std::cos(_box_spawn_angle) * 15.0f, 5.0f, std::sin(_box_spawn_angle) * 15.0f},
          .velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
          .orientation = math::Quatf::axis_angle(math::Vec3f{0.0f, 1.0f, 0.0f}, math::deg_to_rad(90.0f)),
          .angular_velocity = math::Vec3f{0.0f, 0.0f, 0.0f},
      });
      _boxes.emplace_back(new_box);
      if (_boxes.size() == 768) {
        stop();
      }
    }
  }

  void on_stop() final {
    for (auto const box : _boxes) {
      _box_manager->destroy(box);
    }
    _boxes.clear();
  }

private:
  client::Dynamic_prop_manager *_box_manager{};
  util::Allocating_list<client::Dynamic_prop_handle> _boxes{};
  float _box_spawn_timer{};
  float _box_spawn_angle{};
};

class Client : public engine::App {
public:
  Client()
      : App{{
            .world_create_info =
                {
                    .worker_thread_count = 7,
                    .gravitational_acceleration = {0.0f, -9.81f, 0.0f},
                },
            .world_simulate_info = {.delta_time = physics_delta_time, .substep_count = physics_substeps},
            .window_extents = {1280, 720},
            .full_screen = false,
        }} {}

  void pre_loop() final {
    auto const world = get_world();
    auto const graphics = get_graphics();
    auto const scene = get_scene();
    auto const box_mass = 216.0f;
    auto const box_radius = 0.3f;
    auto const box_shape = physics::Box{{box_radius, box_radius, box_radius}};
    _resources = create_resources(graphics);
    _box_manager = std::make_unique<client::Dynamic_prop_manager>(client::Dynamic_prop_manager_create_info{
        .scene = scene,
        .surface_mesh = _resources.cube_mesh.get(),
        .surface_material = _resources.striped_cotton_material,
        .surface_pretransform = math::Mat3x4f{{box_radius, 0.0f, 0.0f, 0.0f},
                                              {0.0f, box_radius, 0.0f, 0.0f},
                                              {0.0f, 0.0f, box_radius, 0.0f}},
        .space = world,
        .body_mass = box_mass,
        .body_inertia_tensor = box_mass * physics::solid_inertia_tensor(box_shape),
        .body_shape = box_shape,
        .body_material =
            {
                .static_friction_coefficient = 0.3f,
                .dynamic_friction_coefficient = 0.2f,
                .restitution_coefficient = 0.1f,
            },
    });
    world->create_static_body({
        .shape = physics::Box{{100.0f, 0.5f, 100.0f}},
        .material =
            {
                .static_friction_coefficient = 0.3f,
                .dynamic_friction_coefficient = 0.2f,
                .restitution_coefficient = 0.1f,
            },
        .position = {0.0f, -0.5f, 0.0f},
    });
    _ground_surface = {
        .mesh = _resources.cube_mesh.get(),
        .material = _resources.ground_material,
        .transform = math::Mat3x4f{{100.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f, -0.5f}, {0.0f, 0.0f, 100.0f, 0.0f}},
    };
    scene->add(&_ground_surface);
    scene->sky_irradiance(mix(graphics::Rgb_spectrum{0.11f}, graphics::Rgb_spectrum{0.0f, 0.0f, 0.33f}, 0.1f));
    scene->ground_albedo(_ground_surface.material.base_color_tint);
    scene->sun(graphics::Directional_light{
        .irradiance = graphics::Rgb_spectrum{1.3f},
        .direction = normalize(math::Vec3f{1.0f, 1.3f, 0.5f}),
    });
    get_camera()->position = {-10.0f, 3.5f, 10.0f};
    get_camera()->zoom = 2.0f * math::Vec2f{9.0f / 16.0f, 1.0f};
    get_camera()->exposure = 1.0f;
    // srand(25);
    _column_phase = Column_phase{_box_manager.get(), &_selection};
    _ring_phase = Ring_phase{_box_manager.get()};
    _pyramid_phase = Pyramid_phase{_box_manager.get()};
    _phases = {&_column_phase, &_ring_phase, &_ring_phase};
    _phase_index = 0;
    _phases[_phase_index]->start();
  }

  // void pre_input() final {
  //   auto const fps = 1.0 / get_loop_iteration_wall_time();
  //   std::cout << (int)fps << " fps\n";
  // }

  void post_input() final {
    auto const window = get_window();
    if (window->should_close()) {
      stop_looping();
      return;
    }
    auto const camera = get_camera();
    if (window->is_mouse_button_pressed(mb_right)) {
      window->set_cursor_mode(engine::Cursor_mode::disabled);
      auto const rotation_matrix = math::Mat3x3f::rotation(camera->orientation);
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
      auto const dt = static_cast<float>(get_loop_iteration_wall_time());
      camera->position += movement * dt;
      _camera_yaw -= window->get_delta_cursor_position().x * 0.002f;
      _camera_pitch -= window->get_delta_cursor_position().y * 0.002f;
      _camera_pitch = std::clamp(_camera_pitch, -0.5f * std::numbers::pi_v<float>, 0.5f * std::numbers::pi_v<float>);
    } else {
      window->set_cursor_mode(engine::Cursor_mode::normal);
    }
    camera->orientation = math::Quatf::axis_angle(math::Vec3f::y_axis(), _camera_yaw) *
                          math::Quatf::axis_angle(math::Vec3f::x_axis(), _camera_pitch);
  }

  void post_physics() final {
    auto const &simulate_result = get_world_simulate_result();
    _max_physics_wall_time = std::max(_max_physics_wall_time, simulate_result.total_wall_time);
    _total_physics_wall_time += simulate_result.total_wall_time;
    _total_physics_simulated_time += physics_delta_time;
    _total_narrowphase_wall_time += simulate_result.narrowphase_wall_time;
    if (_phases[_phase_index]->is_running()) {
      _phases[_phase_index]->post_physics();
      if (!_phases[_phase_index]->is_running()) {
        std::cout << "physics narrowphase wall time: " << _total_narrowphase_wall_time << "\n";
        std::cout << "physics total wall time: " << _total_physics_wall_time << "\n";
        std::cout << "physics max wall time: " << _max_physics_wall_time << "\n";
        std::cout << "physics simulated time: " << _total_physics_simulated_time << "\n";
      }
    } else {
      _max_physics_wall_time = 0.0;
      _total_physics_wall_time = 0.0;
      _total_physics_simulated_time = 0.0;
      _total_narrowphase_wall_time = 0.0;
      _phase_index = (_phase_index + 1) % _phases.size();
      _phases[_phase_index]->start();
    }
    if (simulate_result.total_wall_time > physics_delta_time) {
      std::cout << "SLOWER THAN REAL TIME: " << simulate_result.total_wall_time * 1000.0 << " ms\n";
    }
  }

private:
  Resources _resources;
  std::unique_ptr<client::Dynamic_prop_manager> _box_manager;
  graphics::Surface _ground_surface;
  std::optional<client::Dynamic_prop_handle> _selection;
  float _camera_yaw{math::deg_to_rad(-45.0f)};
  float _camera_pitch{0.0f};
  Column_phase _column_phase;
  Ring_phase _ring_phase;
  Pyramid_phase _pyramid_phase;
  std::array<Phase *, 3> _phases;
  std::size_t _phase_index{};
  double _max_physics_wall_time{0.0};
  double _total_physics_wall_time{0.0};
  double _total_physics_simulated_time{0.0};
  double _total_narrowphase_wall_time{0.0};
};

int main() {
  return Client{}.run();
}
