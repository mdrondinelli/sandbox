#ifndef MARLON_PHYSICS_CONTACT_H
#define MARLON_PHYSICS_CONTACT_H

#include "../math/vec.h"

namespace marlon {
namespace physics {
struct Contact {
  math::Vec3f position;
  math::Vec3f normal;
  float depth;
};
} // namespace physics
} // namespace marlon

#endif