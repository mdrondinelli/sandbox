#ifndef MARLON_CLIENT_SPACE_OBJECT_H
#define MARLON_CLIENT_SPACE_OBJECT_H

#include <cstddef>
#include <cstdint>

#include <functional>
#include <memory>
#include <span>

namespace marlon {
namespace client {
struct Entity_reference {
  std::uint64_t value;
};

struct Entity_create_info {
  void const *parameters;
};

class Entity_manager {
public:
  virtual ~Entity_manager() = default;

  virtual Entity_reference
  create_entity(Entity_create_info const &create_info) = 0;

  virtual void destroy_entity(Entity_reference entity) = 0;

  virtual void tick_entities(float delta_time) = 0;
};

constexpr bool operator==(Entity_reference lhs, Entity_reference rhs) noexcept {
  return lhs.value == rhs.value;
}

class Entity_construction_queue {
public:
  void push(Entity_manager *manager, Entity_create_info const &create_info);

  void consume();

private:
  std::vector<std::pair<Entity_manager *, Entity_create_info>> _elements;
};

class Entity_destruction_queue {
public:
  void push(Entity_manager *manager, Entity_reference entity);

  void consume();

private:
  std::vector<std::pair<Entity_manager *, Entity_reference>> _elements;
};
} // namespace client
} // namespace marlon

namespace std {
template <> struct hash<marlon::client::Entity_reference> {
  std::size_t
  operator()(marlon::client::Entity_reference reference) const noexcept {
    return std::hash<std::uint64_t>{}(reference.value);
  }
};
} // namespace std

#endif
