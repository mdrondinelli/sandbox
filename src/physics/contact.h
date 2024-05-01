#ifndef MARLON_PHYSICS_CONTACT_H
#define MARLON_PHYSICS_CONTACT_H

#include <algorithm>
#include <array>
#include <span>

#include "../math/math.h"

namespace marlon {
namespace physics {
struct Contact {
  math::Vec3f normal;
  std::array<math::Vec3f, 2> local_positions;
  float separation_bias;
};

struct Cached_contact {
  Contact contact;
  std::array<math::Quatf, 2> initial_object_orientations;
};

class Contact_manifold {
public:
  Contact_manifold() noexcept = default;

  void clear() noexcept { _size = 0; }

  void update(std::array<math::Vec3f, 2> const &object_positions,
              std::array<math::Quatf, 2> const &object_orientations) noexcept {
    using namespace math;
    auto constexpr max_position_distance = 0.01f;
    auto constexpr min_orientation_abs_dot = 0.99f;
    auto const object_rotations = std::array<Mat3x3f, 2>{
        Mat3x3f::rotation(object_orientations[0]),
        Mat3x3f::rotation(object_orientations[1]),
    };
    for (auto i = std::size_t{}; i != _size;) {
      auto const &cached_contact = contacts()[i];
      auto const keep = [&] {
        if (abs(dot(object_orientations[0],
                    cached_contact.initial_object_orientations[0])) <
            min_orientation_abs_dot) {
          return false;
        }
        if (abs(dot(object_orientations[1],
                    cached_contact.initial_object_orientations[1])) <
            min_orientation_abs_dot) {
          return false;
        }
        auto const global_positions = std::array<Vec3f, 2>{
            object_positions[0] +
                object_rotations[0] * cached_contact.contact.local_positions[0],
            object_positions[1] +
                object_rotations[1] * cached_contact.contact.local_positions[1],
        };
        return length_squared(global_positions[0] - global_positions[1]) <
               max_position_distance * max_position_distance;
      }();
      if (keep) {
        ++i;
      } else {
        std::shift_left(_contacts.data() + i, _contacts.data() + _size--, 1);
      }
    }
  }

  void insert(Cached_contact const &contact) noexcept {
    auto constexpr max_always_replace_deviation = 0.01f;
    auto const [closest_contact, closest_distance] = closest(contact.contact);
    if (closest_distance < max_always_replace_deviation) {
      *closest_contact = contact;
    } else if (_size < max_size) {
      _contacts[_size++] = contact;
    } else {
      *closest_contact = contact;
    }
  }

  std::span<Cached_contact const> contacts() const noexcept {
    return {_contacts.data(), _size};
  }

  std::span<Cached_contact> contacts() noexcept {
    return {_contacts.data(), _size};
  }

  bool marked() const noexcept { return _marked; }

  void marked(bool marked) noexcept { _marked = marked; }

private:
  static auto constexpr max_size = std::size_t{3};

  std::pair<Cached_contact *, float> closest(Contact const &contact) noexcept {
    auto const begin = _contacts.data();
    auto const end = _contacts.data() + _size;
    auto closest_contact = static_cast<Cached_contact *>(nullptr);
    auto closest_distance = std::numeric_limits<float>::infinity();
    for (auto it = begin; it != end; ++it) {
      auto const distance = length_squared(it->contact.local_positions[0] -
                                           contact.local_positions[0]) +
                            length_squared(it->contact.local_positions[1] -
                                           contact.local_positions[1]);
      if (distance < closest_distance) {
        closest_contact = it;
        closest_distance = distance;
      }
    }
    return {closest_contact, closest_distance};
  }

  std::array<Cached_contact, max_size> _contacts;
  std::uint8_t _size{};
  bool _marked{};
};
} // namespace physics
} // namespace marlon

#endif