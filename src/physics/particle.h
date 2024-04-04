#ifndef MARLON_PHYSICS_PARTICLE_H
#define MARLON_PHYSICS_PARTICLE_H

#include <cstdint>

#include <functional>
#include <span>
#include <unordered_map>

#include "../math/vec.h"
#include "handle.h"
#include "material.h"

namespace marlon {
namespace physics {
class Particle_motion_callback;

struct Particle_handle {
  Object_handle value;
};

struct Particle_create_info {
  Particle_motion_callback *motion_callback{};
  float radius{0.0f};
  float mass{1.0f};
  Material material;
  math::Vec3f position{math::Vec3f::zero()};
  math::Vec3f velocity{math::Vec3f::zero()};
};

class World;

class Particle_motion_callback {
public:
  virtual ~Particle_motion_callback() = default;

  virtual void on_particle_motion(World const &world,
                                  Particle_handle particle) = 0;
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
    return hash<std::size_t>{}(particle_reference.value);
  }
};
} // namespace std

#endif