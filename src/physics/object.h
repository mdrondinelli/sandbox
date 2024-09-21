#ifndef MARLON_PHYSICS_OBJECT_H
#define MARLON_PHYSICS_OBJECT_H

#include <cstdint>

#include <array>
#include <variant>

#include "math/vec.h"
#include "util/hash.h"

namespace marlon {
namespace physics {
using Object_handle = std::uint32_t;

enum class Object_type : std::uint8_t { particle = 1, rigid_body, static_body };

auto constexpr object_handle_type_bits = 2;
auto constexpr object_handle_index_bits = 32 - object_handle_type_bits;
auto constexpr object_handle_index_mask = std::uint32_t{0xffffffffu} >> object_handle_type_bits;

class Particle;
class Rigid_body;
class Static_body;

struct Object_integrate_info {
  math::Vec3f delta_velocity;
  float delta_time;
  float damping_factor;
  float motion_smoothing_factor;
  float motion_limit;
};

class Object {
public:
  Object() = default;

  explicit constexpr Object(Object_handle handle) noexcept
      : _handle{handle} {}

  constexpr operator bool() const noexcept {
    return _handle != 0;
  }

  constexpr Object_handle handle() const noexcept {
    return _handle;
  }

  constexpr Object_type type() const noexcept {
    return static_cast<Object_type>(_handle >> object_handle_index_bits);
  }

  constexpr int index() const noexcept {
    return static_cast<int>(_handle & object_handle_index_mask);
  }

  constexpr std::variant<Particle, Rigid_body, Static_body> specific() const noexcept;

  friend constexpr bool operator==(Object lhs, Object rhs) noexcept = default;

private:
  std::uint32_t _handle{};
};

class Particle {
public:
  Particle() = default;

  constexpr explicit Particle(int index) noexcept
      : Particle{Object{static_cast<Object_handle>(Object_type::particle) << object_handle_index_bits |
                        static_cast<Object_handle>(index)}} {}

  constexpr explicit Particle(Object generic) noexcept
      : _generic{generic} {}

  constexpr int index() const noexcept {
    return _generic.index();
  }

  constexpr Object generic() const noexcept {
    return _generic;
  }

  friend constexpr bool operator==(Particle lhs, Particle rhs) noexcept = default;

private:
  Object _generic;
};

class Rigid_body {
public:
  Rigid_body() = default;

  constexpr explicit Rigid_body(int index) noexcept
      : Rigid_body{Object{static_cast<Object_handle>(Object_type::rigid_body) << object_handle_index_bits |
                          static_cast<Object_handle>(index)}} {}

  constexpr explicit Rigid_body(Object generic) noexcept
      : _generic{generic} {}

  constexpr Object_handle index() const noexcept {
    return _generic.index();
  }

  constexpr Object generic() const noexcept {
    return _generic;
  }

  friend constexpr bool operator==(Rigid_body lhs, Rigid_body rhs) noexcept = default;

private:
  Object _generic;
};

class Static_body {
public:
  Static_body() = default;

  constexpr explicit Static_body(int index) noexcept
      : Static_body{Object{static_cast<Object_handle>(Object_type::static_body) << object_handle_index_bits |
                           static_cast<Object_handle>(index)}} {}

  constexpr explicit Static_body(Object generic) noexcept
      : _generic{generic} {}

  constexpr Object_handle index() const noexcept {
    return _generic.index();
  }

  constexpr Object generic() const noexcept {
    return _generic;
  }

  friend constexpr bool operator==(Static_body lhs, Static_body rhs) noexcept = default;

private:
  Object _generic;
};

class Object_pair {
public:
  constexpr Object_pair(Particle first, Particle second)
      : Object_pair{first.generic(), second.generic()} {}

  constexpr Object_pair(Particle first, Rigid_body second)
      : _objects{first.generic(), second.generic()} {}

  constexpr Object_pair(Particle first, Static_body second)
      : _objects{first.generic(), second.generic()} {}

  constexpr Object_pair(Rigid_body first, Particle second)
      : _objects{second.generic(), first.generic()} {}

  constexpr Object_pair(Rigid_body first, Rigid_body second)
      : Object_pair{first.generic(), second.generic()} {}

  constexpr Object_pair(Rigid_body first, Static_body second)
      : _objects{first.generic(), second.generic()} {}

  constexpr Object_pair(Static_body first, Particle second)
      : _objects{second.generic(), first.generic()} {}

  constexpr Object_pair(Static_body first, Rigid_body second)
      : _objects{second.generic(), first.generic()} {}

  constexpr Object first_generic() const noexcept {
    return _objects[0];
  }

  constexpr Object second_generic() const noexcept {
    return _objects[1];
  }

  constexpr std::variant<Particle, Rigid_body> first_specific() const noexcept {
    return std::visit(
        [&](auto const object) -> std::variant<Particle, Rigid_body> {
          using T = std::decay_t<decltype(object)>;
          if constexpr (std::is_same_v<T, Particle> || std::is_same_v<T, Rigid_body>) {
            return object;
          } else {
            math::unreachable();
            throw;
          }
        },
        _objects[0].specific());
  }

  constexpr std::variant<Particle, Rigid_body, Static_body> second_specific() const noexcept {
    return _objects[1].specific();
  }

  constexpr std::variant<std::pair<Particle, Particle>,
                         std::pair<Particle, Rigid_body>,
                         std::pair<Particle, Static_body>,
                         std::pair<Rigid_body, Rigid_body>,
                         std::pair<Rigid_body, Static_body>>
  specific() const noexcept {
    return std::visit(
        [&](auto const first, auto const second) -> std::variant<std::pair<Particle, Particle>,
                                                                 std::pair<Particle, Rigid_body>,
                                                                 std::pair<Particle, Static_body>,
                                                                 std::pair<Rigid_body, Rigid_body>,
                                                                 std::pair<Rigid_body, Static_body>> {
          using T = std::decay_t<decltype(first)>;
          using U = std::decay_t<decltype(second)>;
          if constexpr ((std::is_same_v<T, Particle> && std::is_same_v<U, Particle>) ||
                        (std::is_same_v<T, Particle> && std::is_same_v<U, Rigid_body>) ||
                        (std::is_same_v<T, Particle> && std::is_same_v<U, Static_body>) ||
                        (std::is_same_v<T, Rigid_body> && std::is_same_v<U, Rigid_body>) ||
                        (std::is_same_v<T, Rigid_body> && std::is_same_v<U, Static_body>)) {
            return std::pair{first, second};
          } else {
            math::unreachable();
            throw;
          }
        },
        _objects[0].specific(),
        _objects[1].specific());
  }

  friend constexpr bool operator==(Object_pair lhs, Object_pair rhs) noexcept = default;

private:
  constexpr Object_pair(Object first, Object second) noexcept
      : _objects{first, second} {
    if (_objects[1].handle() < _objects[0].handle()) {
      std::swap(_objects[1], _objects[0]);
    }
  }

  std::array<Object, 2> _objects;
};

constexpr std::variant<Particle, Rigid_body, Static_body> Object::specific() const noexcept {
  switch (type()) {
  case Object_type::particle:
    return Particle{*this};
  case Object_type::rigid_body:
    return Rigid_body{*this};
  case Object_type::static_body:
    return Static_body{*this};
  default:
    math::unreachable();
  }
}
} // namespace physics
namespace util {
template <> struct Hash<physics::Object_pair> {
  constexpr std::size_t operator()(physics::Object_pair x) const noexcept {
    return (static_cast<std::size_t>(x.first_generic().handle()) << 32) | x.second_generic().handle();
  }
};
} // namespace util
} // namespace marlon
#endif
