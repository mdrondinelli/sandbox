#ifndef MARLON_PHYSICS_OBJECT_H
#define MARLON_PHYSICS_OBJECT_H

#include <cstdint>

#include <concepts>
#include <type_traits>

namespace marlon {
namespace physics {
enum class Object_type : std::uint8_t { particle, rigid_body, static_body };

using Object_handle = std::uint32_t;

auto constexpr object_handle_type_bits = 2;
auto constexpr object_handle_index_bits = 32 - object_handle_type_bits;
auto constexpr object_handle_index_mask =
    std::uint32_t{0xffffffffu} >> object_handle_type_bits;

class Object_pair {
public:
  explicit Object_pair(std::uint64_t id) noexcept
      : _objects{
            static_cast<Object_handle>(id >> 32),
            static_cast<Object_handle>(id),
        } {}

  explicit Object_pair(
      std::pair<Object_handle, Object_handle> objects) noexcept {
    if (objects.first < objects.second) {
      _objects[0] = objects.first;
      _objects[1] = objects.second;
    } else {
      _objects[0] = objects.second;
      _objects[1] = objects.first;
    }
  }

  Object_handle first() const noexcept { return _objects[0]; }

  Object_handle second() const noexcept { return _objects[1]; }

  Object_handle other(Object_handle object) const noexcept {
    return _objects[_objects[0] == object ? 1 : 0];
  }

  std::uint64_t id() const noexcept {
    return static_cast<std::uint64_t>(_objects[0]) << 32 | _objects[1];
  }

private:
  std::array<Object_handle, 2> _objects;
};

class Particle_handle {
public:
  Particle_handle() = default;

  constexpr explicit Particle_handle(Object_handle index) noexcept
      : _value{static_cast<Object_handle>(Object_type::particle)
                   << object_handle_index_bits |
               index} {}

  constexpr Object_handle value() const noexcept { return _value; }

  constexpr Object_handle index() const noexcept {
    return _value & object_handle_index_mask;
  }

private:
  Object_handle _value;
};

class Rigid_body_handle {
public:
  Rigid_body_handle() = default;

  constexpr explicit Rigid_body_handle(Object_handle index) noexcept
      : _value{static_cast<Object_handle>(Object_type::rigid_body)
                   << object_handle_index_bits |
               index} {}

  constexpr Object_handle value() const noexcept { return _value; }

  constexpr Object_handle index() const noexcept {
    return _value & object_handle_index_mask;
  }

private:
  Object_handle _value;
};

class Static_body_handle {
public:
  Static_body_handle() = default;

  constexpr explicit Static_body_handle(Object_handle index) noexcept
      : _value{static_cast<Object_handle>(Object_type::static_body)
                   << object_handle_index_bits |
               index} {}

  constexpr Object_handle value() const noexcept { return _value; }

  constexpr Object_handle index() const noexcept {
    return _value & object_handle_index_mask;
  }

private:
  Object_handle _value;
};

constexpr bool operator==(Particle_handle lhs, Particle_handle rhs) noexcept {
  return lhs.value() == rhs.value();
}

constexpr bool operator==(Rigid_body_handle lhs,
                          Rigid_body_handle rhs) noexcept {
  return lhs.value() == rhs.value();
}

constexpr bool operator==(Static_body_handle lhs,
                          Static_body_handle rhs) noexcept {
  return lhs.value() == rhs.value();
}

constexpr Object_type get_object_type(Object_handle object) noexcept {
  return static_cast<Object_type>(object >> object_handle_index_bits);
}

template <typename F>
  requires std::same_as<std::invoke_result_t<F, Particle_handle>,
                        std::invoke_result_t<F, Rigid_body_handle>> &&
           std::same_as<std::invoke_result_t<F, Particle_handle>,
                        std::invoke_result_t<F, Static_body_handle>>
std::invoke_result_t<F, Particle_handle> visit(F &&f, Object_handle object) {
  switch (get_object_type(object)) {
  case Object_type::particle:
    return f(Particle_handle{object});
  case Object_type::rigid_body:
    return f(Rigid_body_handle{object});
  case Object_type::static_body:
    return f(Static_body_handle{object});
  default:
    math::unreachable();
    throw;
  }
}
} // namespace physics
} // namespace marlon
namespace std {
template <> struct hash<marlon::physics::Particle_handle> {
  std::size_t operator()(
      marlon::physics::Particle_handle particle_reference) const noexcept {
    return hash<std::size_t>{}(particle_reference.value());
  }
};

template <> struct hash<marlon::physics::Rigid_body_handle> {
  std::size_t
  operator()(marlon::physics::Rigid_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value());
  }
};

template <> struct hash<marlon::physics::Static_body_handle> {
  std::size_t
  operator()(marlon::physics::Static_body_handle reference) const noexcept {
    return hash<std::size_t>{}(reference.value());
  }
};
} // namespace std
#endif