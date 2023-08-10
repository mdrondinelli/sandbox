#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <functional>
#include <span>
#include <unordered_map>

#include "../math/vec.h"
#include "material.h"

namespace marlon {
namespace physics {
class Particle_motion_callback;

struct Particle_handle {
  std::uint64_t value;
};

struct Particle_create_info {
  Particle_motion_callback *motion_callback{nullptr};
  std::uint64_t collision_flags{0};
  std::uint64_t collision_mask{0};
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
  float mass{1.0f};
  float radius{0.0f};
  Material material;
};

struct Particle_motion_event {
  Particle_handle particle;
  math::Vec3f position;
  math::Vec3f velocity;
};

struct Particle_contact {
  math::Vec3f normal;
  float separation;
};

class Particle_construction_queue {
public:
  void push(Particle_create_info const &create_info) {
    create_infos.push_back(create_info);
  }

  std::span<Particle_create_info const> get() const noexcept {
    return create_infos;
  }

private:
  std::vector<Particle_create_info> create_infos;
};

class Particle_destruction_queue {
public:
  void push(Particle_handle reference) { references.push_back(reference); }

  std::span<Particle_handle const> get() const noexcept { return references; }

private:
  std::vector<Particle_handle> references;
};

class Particle_motion_callback {
public:
  virtual ~Particle_motion_callback() = default;

  virtual void on_particle_motion(Particle_motion_event const &event) = 0;
};

constexpr bool operator==(Particle_handle lhs, Particle_handle rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Particle_handle> {
  std::size_t operator()(
      marlon::physics::Particle_handle particle_reference) const noexcept {
    return hash<std::uint64_t>{}(particle_reference.value);
  }
};
} // namespace std

#endif