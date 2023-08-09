#ifndef MARLON_PHYSICS_CONTACT_H
#define MARLON_PHYSICS_CONTACT_H

#include "../math/vec.h"

namespace marlon {
namespace physics {
struct Particle_contact {
  math::Vec3f normal;
  float depth;
};
} // namespace physics
} // namespace marlon

#endif