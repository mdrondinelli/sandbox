#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <functional>

#include "../math/vec.h"

namespace marlon {
namespace physics {
struct Particle_reference {
  std::uint64_t value;
};

struct Particle_create_info {
  math::Vec3f position;
  math::Vec3f velocity;
  float mass;
  Particle_reference *out_reference;
};

struct Particle_motion_event {
  Particle_reference reference;
  math::Vec3f position;
  math::Vec3f velocity;
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