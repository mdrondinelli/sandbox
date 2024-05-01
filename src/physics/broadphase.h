#ifndef MARLON_PHYSICS_BROADPHASE_H
#define MARLON_PHYSICS_BROADPHASE_H

#include "aabb_tree.h"
#include "object.h"

namespace marlon {
namespace physics {
using Broadphase_bvh = Aabb_tree<Object>;
}
} // namespace marlon

#endif