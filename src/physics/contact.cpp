#include "contact.h"

namespace marlon::physics {
using namespace math;

namespace {
struct Object_position_solve_data {
  Mat3x4f transform;
  Mat3x4f inverse_transform;
  float inverse_mass;
  Mat3x3f inverse_inertia_tensor;
};

Object_position_solve_data
position_solve_data(Particle_data const &data) noexcept {
  return Object_position_solve_data{
      .transform = Mat3x4f::translation(data.position()),
      .inverse_transform = Mat3x4f::translation(-data.position()),
      .inverse_mass = data.inverse_mass(),
      .inverse_inertia_tensor = Mat3x3f::zero(),
  };
}

Object_position_solve_data
position_solve_data(Rigid_body_data const &data) noexcept {
  auto const transform = Mat3x4f::rigid(data.position(), data.orientation());
  return Object_position_solve_data{
      .transform = transform,
      .inverse_transform = rigid_inverse(transform),
      .inverse_mass = data.inverse_mass(),
      .inverse_inertia_tensor = data.inverse_inertia_tensor(),
  };
}

Object_position_solve_data
position_solve_data(Static_body_data const &data) noexcept {
  auto const transform = Mat3x4f::rigid(data.position(), data.orientation());
  return Object_position_solve_data{
      .transform = transform,
      .inverse_transform = rigid_inverse(transform),
      .inverse_mass = 0.0f,
      .inverse_inertia_tensor = Mat3x3f::zero(),
  };
}

void apply_positional_impulse(Particle_data *object_data,
                              Object_position_solve_data const &derived_data,
                              Vec3f const & /*local_position*/,
                              Vec3f const & /*local_impulse*/,
                              Vec3f const &global_impulse) noexcept {
  object_data->position(object_data->position() +
                        global_impulse * derived_data.inverse_mass);
}

void apply_positional_impulse(Rigid_body_data *object_data,
                              Object_position_solve_data const &derived_data,
                              Vec3f const &local_position,
                              Vec3f const &local_impulse,
                              Vec3f const &global_impulse) noexcept {
  auto const rotation = Mat3x3f{{derived_data.transform[0][0],
                                 derived_data.transform[0][1],
                                 derived_data.transform[0][2]},
                                {derived_data.transform[1][0],
                                 derived_data.transform[1][1],
                                 derived_data.transform[1][2]},
                                {derived_data.transform[2][0],
                                 derived_data.transform[2][1],
                                 derived_data.transform[2][2]}};
  auto const rotated_inverse_inertia_tensor =
      rotation * derived_data.inverse_inertia_tensor;
  object_data->position(object_data->position() +
                        global_impulse * derived_data.inverse_mass);
  object_data->orientation(object_data->orientation() +
                           0.5f *
                               Quatf{0.0f,
                                     rotated_inverse_inertia_tensor *
                                         cross(local_position, local_impulse)} *
                               object_data->orientation());
}

void apply_positional_impulse(
    Static_body_data * /*object_data*/,
    Object_position_solve_data const & /*derived_data*/,
    Vec3f const & /*local_position*/,
    Vec3f const & /*local_impulse*/,
    Vec3f const & /*global_impulse*/) noexcept {}

float generalized_inverse_mass(float inverse_mass,
                               Mat3x3f const &inverse_inertia_tensor,
                               Vec3f const &impulse_position,
                               Vec3f const &impulse_direction) noexcept {
  auto const rxn = cross(impulse_position, impulse_direction);
  return inverse_mass + dot(rxn, inverse_inertia_tensor * rxn);
}
} // namespace

void Contact::solve_position(Object_pair objects,
                             Object_storage &storage) noexcept {
  std::visit(
      [&](auto const objects) {
        auto const object_data = std::pair{storage.data(objects.first),
                                           storage.data(objects.second)};
        auto const object_derived_data =
            std::array<Object_position_solve_data, 2>{
                position_solve_data(*object_data.first),
                position_solve_data(*object_data.second),
            };
        auto const contact_positions = std::array<Vec3f, 2>{
            object_derived_data[0].transform * Vec4f{local_positions[0], 1.0f},
            object_derived_data[1].transform * Vec4f{local_positions[1], 1.0f},
        };
        auto const relative_position =
            contact_positions[0] - contact_positions[1];
        auto const separation =
            dot(relative_position, normal) + initial_separation;
        // if (separation >= 0.0f) {
        //   return;
        // }
        auto const local_contact_normals = std::array<Vec3f, 2>{
            object_derived_data[0].inverse_transform * Vec4f{normal, 0.0f},
            object_derived_data[1].inverse_transform * Vec4f{normal, 0.0f},
        };
        auto const generalized_inverse_masses = std::array<float, 2>{
            generalized_inverse_mass(
                object_derived_data[0].inverse_mass,
                object_derived_data[0].inverse_inertia_tensor,
                local_positions[0],
                local_contact_normals[0]),
            generalized_inverse_mass(
                object_derived_data[1].inverse_mass,
                object_derived_data[1].inverse_inertia_tensor,
                local_positions[1],
                local_contact_normals[1]),
        };
        auto const impulse_scalar =
            0.8f * max(-separation / (generalized_inverse_masses[0] +
                                      generalized_inverse_masses[1]),
                       -impulse);
        impulse += impulse_scalar;
        auto const local_impulses = std::array<Vec3f, 2>{
            impulse_scalar * local_contact_normals[0],
            -impulse_scalar * local_contact_normals[1],
        };
        auto const global_impulse = impulse_scalar * normal;
        apply_positional_impulse(object_data.first,
                                 object_derived_data[0],
                                 local_positions[0],
                                 local_impulses[0],
                                 global_impulse);
        apply_positional_impulse(object_data.second,
                                 object_derived_data[1],
                                 local_positions[1],
                                 local_impulses[1],
                                 -global_impulse);
      },
      objects.specific());
}

namespace {
struct Object_velocity_solve_data {
  Vec3f velocity;
  Vec3f angular_velocity;
  Mat3x3f rotation;
  Mat3x3f inverse_rotation;
  float inverse_mass;
  Mat3x3f inverse_inertia_tensor;
  Material material;
};

Object_velocity_solve_data
velocity_solve_data(Particle_data const &data) noexcept {
  return {
      .velocity = data.velocity(),
      .angular_velocity = Vec3f::zero(),
      .rotation = Mat3x3f::identity(),
      .inverse_rotation = Mat3x3f::identity(),
      .inverse_mass = data.inverse_mass(),
      .inverse_inertia_tensor = Mat3x3f::zero(),
      .material = data.material(),
  };
}

Object_velocity_solve_data
velocity_solve_data(Rigid_body_data const &data) noexcept {
  auto const rotation = Mat3x3f::rotation(data.orientation());
  return {
      .velocity = data.velocity(),
      .angular_velocity = data.angular_velocity(),
      .rotation = rotation,
      .inverse_rotation = transpose(rotation),
      .inverse_mass = data.inverse_mass(),
      .inverse_inertia_tensor = data.inverse_inertia_tensor(),
      .material = data.material(),
  };
}

Object_velocity_solve_data
velocity_solve_data(Static_body_data const &data) noexcept {
  auto const rotation = Mat3x3f::rotation(data.orientation());
  return {
      .velocity = Vec3f::zero(),
      .angular_velocity = Vec3f::zero(),
      .rotation = rotation,
      .inverse_rotation = transpose(rotation),
      .inverse_mass = 0.0f,
      .inverse_inertia_tensor = Mat3x3f::zero(),
      .material = data.material(),
  };
}

void apply_velocity_impulse(Particle_data *object_data,
                            Object_velocity_solve_data const &derived_data,
                            Vec3f const & /*local_position*/,
                            Vec3f const & /*local_impulse*/,
                            Vec3f const &global_impulse) noexcept {
  object_data->velocity(object_data->velocity() +
                        derived_data.inverse_mass * global_impulse);
}

void apply_velocity_impulse(Rigid_body_data *object_data,
                            Object_velocity_solve_data const &derived_data,
                            Vec3f const &local_position,
                            Vec3f const &local_impulse,
                            Vec3f const &global_impulse) noexcept {
  object_data->velocity(object_data->velocity() +
                        derived_data.inverse_mass * global_impulse);
  object_data->angular_velocity(object_data->angular_velocity() +
                                derived_data.rotation *
                                    (derived_data.inverse_inertia_tensor *
                                     cross(local_position, local_impulse)));
}

void apply_velocity_impulse(Static_body_data * /*object_data*/,
                            Object_velocity_solve_data const & /*derived_data*/,
                            Vec3f const & /*local_position*/,
                            Vec3f const & /*local_impulse*/,
                            Vec3f const & /*global_impulse*/) noexcept {}

} // namespace

void Contact::solve_velocity(Object_pair objects,
                             Object_storage &storage,
                             float restitution_epsilon) noexcept {
  std::visit(
      [&](auto const objects) {
        auto const object_data = std::pair{storage.data(objects.first),
                                           storage.data(objects.second)};
        auto const object_derived_data =
            std::array<Object_velocity_solve_data, 2>{
                velocity_solve_data(*object_data.first),
                velocity_solve_data(*object_data.second),
            };
        auto const relative_contact_positions = std::array<Vec3f, 2>{
            object_derived_data[0].rotation * local_positions[0],
            object_derived_data[1].rotation * local_positions[1],
        };
        auto const relative_velocity =
            (object_derived_data[0].velocity +
             cross(object_derived_data[0].angular_velocity,
                   relative_contact_positions[0])) -
            (object_derived_data[1].velocity +
             cross(object_derived_data[1].angular_velocity,
                   relative_contact_positions[1]));
        auto const separating_velocity = dot(relative_velocity, normal);
        if (separating_velocity >= 0.0f) {
          return;
        }
        auto const local_contact_normals = std::array<Vec3f, 2>{
            object_derived_data[0].inverse_rotation * normal,
            object_derived_data[1].inverse_rotation * normal,
        };
        auto const normal_generalized_inverse_masses = std::array<float, 2>{
            generalized_inverse_mass(
                object_derived_data[0].inverse_mass,
                object_derived_data[0].inverse_inertia_tensor,
                local_positions[0],
                local_contact_normals[0]),
            generalized_inverse_mass(
                object_derived_data[1].inverse_mass,
                object_derived_data[1].inverse_inertia_tensor,
                local_positions[1],
                local_contact_normals[1]),
        };
        auto const restitution_coefficient =
            -separating_velocity > restitution_epsilon
                ? 0.5f *
                      (object_derived_data[0].material.restitution_coefficient +
                       object_derived_data[1].material.restitution_coefficient)
                : 0.0f;
        auto const normal_impulse_magnitude =
            (-separating_velocity * (1.0f + restitution_coefficient)) /
            (normal_generalized_inverse_masses[0] +
             normal_generalized_inverse_masses[1]);
        auto local_impulses = std::array<Vec3f, 2>{
            normal_impulse_magnitude * local_contact_normals[0],
            -normal_impulse_magnitude * local_contact_normals[1],
        };
        auto global_impulse = normal_impulse_magnitude * normal;
        auto const tangential_velocity =
            relative_velocity - separating_velocity * normal;
        if (tangential_velocity != Vec3f::zero()) {
          auto const tangential_speed = length(tangential_velocity);
          auto const global_tangent = tangential_velocity / tangential_speed;
          auto const local_tangents = std::array<Vec3f, 2>{
              object_derived_data[0].inverse_rotation * global_tangent,
              object_derived_data[1].inverse_rotation * global_tangent,
          };
          auto const tangential_generalized_inverse_masses =
              std::array<float, 2>{
                  generalized_inverse_mass(
                      object_derived_data[0].inverse_mass,
                      object_derived_data[0].inverse_inertia_tensor,
                      local_positions[0],
                      -local_tangents[0]),
                  generalized_inverse_mass(
                      object_derived_data[1].inverse_mass,
                      object_derived_data[1].inverse_inertia_tensor,
                      local_positions[1],
                      -local_tangents[1]),
              };
          auto const inverse_sum_tangential_generalized_inverse_mass =
              1.0f / (tangential_generalized_inverse_masses[0] +
                      tangential_generalized_inverse_masses[1]);
          auto const static_friction_impulse_magnitude =
              tangential_speed *
              inverse_sum_tangential_generalized_inverse_mass;
          auto const static_friction_coefficient =
              0.5f *
              (object_derived_data[0].material.static_friction_coefficient +
               object_derived_data[1].material.dynamic_friction_coefficient);
          if (static_friction_impulse_magnitude <=
              static_friction_coefficient * normal_impulse_magnitude) {
            local_impulses[0] -=
                static_friction_impulse_magnitude * local_tangents[0];
            local_impulses[1] +=
                static_friction_impulse_magnitude * local_tangents[1];
            global_impulse -=
                static_friction_impulse_magnitude * global_tangent;
          } else {
            auto const dynamic_friction_coefficient =
                0.5f *
                (object_derived_data[0].material.dynamic_friction_coefficient +
                 object_derived_data[1].material.dynamic_friction_coefficient);
            auto const dynamic_friction_impulse_magnitude =
                dynamic_friction_coefficient * normal_impulse_magnitude;
            local_impulses[0] -=
                dynamic_friction_impulse_magnitude * local_tangents[0];
            local_impulses[1] +=
                dynamic_friction_impulse_magnitude * local_tangents[1];
            global_impulse -=
                dynamic_friction_impulse_magnitude * global_tangent;
          }
        }
        apply_velocity_impulse(object_data.first,
                               object_derived_data[0],
                               local_positions[0],
                               local_impulses[0],
                               global_impulse);
        apply_velocity_impulse(object_data.second,
                               object_derived_data[1],
                               local_positions[1],
                               local_impulses[1],
                               -global_impulse);
      },
      objects.specific());
}
} // namespace marlon::physics
