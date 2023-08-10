#ifndef MARLON_PHYSICS_MATERIAL_H
#define MARLON_PHYSICS_MATERIAL_H

namespace marlon {
namespace physics {
struct Material {
  float static_friction_coefficient{};
  float dynamic_friction_coefficient{};
  float restitution_coefficient{};
};
} // namespace physics
} // namespace marlon

#endif