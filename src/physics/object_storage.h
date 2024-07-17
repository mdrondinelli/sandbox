#ifndef MARLON_PHYSICS_OBJECT_STORAGE_H
#define MARLON_PHYSICS_OBJECT_STORAGE_H

#include "particle.h"
#include "rigid_body.h"
#include "static_body.h"

namespace marlon::physics {
class Object_storage {
  using Allocator = util::Stack_allocator<>;

public:
  static constexpr util::Size
  memory_requirement(util::Size max_particles, util::Size max_rigid_bodies, util::Size max_static_bodies) {
    return util::memory_requirement<Allocator, decltype(_particles), decltype(_rigid_bodies), decltype(_static_bodies)>(
        std::tuple{max_particles}, std::tuple{max_rigid_bodies}, std::tuple{max_static_bodies});
  }

  template <typename Allocator>
  static std::pair<util::Block, Object_storage>
  make(Allocator &&allocator, util::Size max_particles, util::Size max_rigid_bodies, util::Size max_static_bodies) {
    auto const block = allocator.alloc(memory_requirement(max_particles, max_rigid_bodies, max_static_bodies));
    return {block, Object_storage{block, max_particles, max_rigid_bodies, max_static_bodies}};
  }

  constexpr Object_storage() noexcept = default;

  explicit Object_storage(util::Block block,
                          util::Size max_particles,
                          util::Size max_rigid_bodies,
                          util::Size max_static_bodies) noexcept {
    util::assign_merged(Allocator{block},
                        std::tie(_particles, _rigid_bodies, _static_bodies),
                        std::tuple{max_particles},
                        std::tuple{max_rigid_bodies},
                        std::tuple{max_static_bodies});
  }

  template <typename F> void for_each_particle(F &&f) { _particles.for_each(f); }

  template <typename F> void for_each_rigid_body(F &&f) { _rigid_bodies.for_each(f); }

  template <typename... Args> Particle create_particle(Args &&...args) {
    return _particles.create(std::forward<Args>(args)...);
  }

  template <typename... Args> Rigid_body create_rigid_body(Args &&...args) {
    return _rigid_bodies.create(std::forward<Args>(args)...);
  }

  template <typename... Args> Static_body create_static_body(Args &&...args) {
    return _static_bodies.create(std::forward<Args>(args)...);
  }

  void destroy(Particle object) { return _particles.destroy(object); }

  void destroy(Rigid_body object) { return _rigid_bodies.destroy(object); }

  void destroy(Static_body object) { return _static_bodies.destroy(object); }

  Particle_data const *data(Particle particle) const noexcept { return _particles.data(particle); }

  Particle_data *data(Particle particle) noexcept { return _particles.data(particle); }

  Rigid_body_data const *data(Rigid_body rigid_body) const noexcept { return _rigid_bodies.data(rigid_body); }

  Rigid_body_data *data(Rigid_body rigid_body) noexcept { return _rigid_bodies.data(rigid_body); }

  Static_body_data const *data(Static_body static_body) const noexcept { return _static_bodies.data(static_body); }

  Static_body_data *data(Static_body static_body) noexcept { return _static_bodies.data(static_body); }

private:
  Particle_storage _particles;
  Rigid_body_storage _rigid_bodies;
  Static_body_storage _static_bodies;
};
} // namespace marlon::physics

#endif