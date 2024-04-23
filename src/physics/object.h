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

constexpr Object_type get_object_type(Object_handle object) noexcept {
  return static_cast<Object_type>(object >> object_handle_index_bits);
}

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

#endif