#ifndef MARLON_PHYSICS_CONSTRAINT_H
#define MARLON_PHYSICS_CONSTRAINT_H

#include <array>

#include <math/vec.h>

#include "object.h"
#include "object_storage.h"

namespace marlon::physics {
class Constraint {
public:
  virtual ~Constraint() {}

  virtual void solve_position(Object_pair objects, Object_storage &object_storage) noexcept = 0;

  virtual void
  solve_velocity(Object_pair objects, Object_storage &object_storage, float restitution_epsilon) noexcept = 0;
};
} // namespace marlon::physics

#endif