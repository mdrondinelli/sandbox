#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <functional>
#include <span>
#include <unordered_map>

#include "../math/vec.h"

namespace marlon {
namespace physics {
class Particle_motion_callback;

struct Particle_reference {
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
  float static_friction_coefficient{0.0f};
  float dynamic_friction_coefficient{0.0f};
  float restitution_coefficient{0.0f};
};

struct Particle_motion_event {
  Particle_reference particle;
  math::Vec3f position;
  math::Vec3f velocity;
};

struct Particle_contact {
  math::Vec3f normal;
  float separation;
};

class Particle_construction_queue {
public:
  void push(Particle_create_info const &create_info);

  std::span<Particle_create_info const> get() const noexcept;

private:
  std::vector<Particle_create_info> create_infos;
};

class Particle_destruction_queue {
public:
  void push(Particle_reference particle);

  std::span<Particle_reference const> get() const noexcept;

private:
  std::vector<Particle_reference> references;
};

class Particle_motion_callback {
public:
  virtual ~Particle_motion_callback() = default;

  virtual void on_particle_motion(Particle_motion_event const &event) = 0;
};

constexpr bool operator==(Particle_reference lhs,
                          Particle_reference rhs) noexcept {
  return lhs.value == rhs.value;
}
} // namespace physics
} // namespace marlon

namespace std {
template <> struct hash<marlon::physics::Particle_reference> {
  std::size_t operator()(
      marlon::physics::Particle_reference particle_reference) const noexcept {
    return hash<std::uint64_t>{}(particle_reference.value);
  }
};
} // namespace std

#endif