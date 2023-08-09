#include "space.h"

#include <array>
#include <iostream>
#include <utility>
#include <variant>

#include <ankerl/unordered_dense.h>

#include "bounding_box.h"

namespace marlon {
namespace physics {
namespace {
// LeafPayload must be nothrow copyable
template <typename LeafPayload> class Bounding_box_hierarchy {
public:
  struct Node {
    Node *parent;
    Bounding_box bounds;
    std::variant<std::array<Node *, 2>, LeafPayload> payload;
  };

  Node *create_leaf(Bounding_box const &bounds, LeafPayload const &payload) {
    auto const node = _node_pool.alloc();
    try {
      _leaf_nodes.emplace(node);
    } catch (...) {
      _node_pool.free(node);
    }
    node->parent = nullptr;
    node->bounds = bounds;
    node->payload = payload;
    return node;
  }

  void destroy_leaf(Node *node) noexcept {
    if (node->parent != nullptr) {
      auto &parents_children = std::get<0>(node->parent->payload);
      parents_children[parents_children[0] == node ? 0 : 1] = nullptr;
    }
    _leaf_nodes.erase(node);
    _node_pool.free(node);
  }

  void build() {
    if (_root_node != nullptr) {
      free_internal_nodes(_root_node);
      _root_node = nullptr;
    }
    if (_leaf_nodes.size() > 1) {
      _flattened_leaf_nodes.clear();
      _flattened_leaf_nodes.reserve(_leaf_nodes.size());
      for (auto const element : _leaf_nodes) {
        _flattened_leaf_nodes.emplace_back(element);
      }
      _root_node = build_internal_node(_flattened_leaf_nodes);
    } else if (_leaf_nodes.size() == 1) {
      for (auto const element : _leaf_nodes) {
        _root_node = element;
      }
    }
  }

  template <typename F>
  void for_each_overlapping_leaf_pair(F &&f) noexcept(
      noexcept(f(std::declval<LeafPayload>(), std::declval<LeafPayload>()))) {
    if (_root_node != nullptr) {
      for_each_overlapping_leaf_pair(_root_node, std::forward<F>(f));
    }
  }

private:
  class Node_pool {
    static constexpr auto initial_segment_size = std::size_t{2048};

  public:
    Node_pool()
        : _segments{Segment{initial_segment_size}},
          _capacity{initial_segment_size} {}

    Node_pool(const Node_pool &other) = delete;

    Node_pool &operator=(const Node_pool &other) = delete;

    Node *alloc() {
      for (auto &segment : _segments) {
        auto const node = segment.alloc();
        if (node != nullptr) {
          return node;
        }
      }
      auto const growth_factor = 2;
      _segments.emplace_back(Segment{(growth_factor - 1) * _capacity});
      _capacity *= growth_factor;
      return _segments.back().alloc();
    }

    void free(Node *node) noexcept {
      for (auto &segment : _segments) {
        if (segment.owns(node)) {
          segment.free(node);
          return;
        }
      }
      assert(false);
    }

  private:
    class Segment {
    public:
      explicit Segment(std::size_t size) {
        nodes.resize(size);
        free_indices.reserve(size);
        for (auto i = std::size_t{}; i < size; ++i) {
          free_indices.emplace_back(static_cast<std::ptrdiff_t>(i));
        }
      }

      Node *alloc() {
        if (!free_indices.empty()) {
          auto const index = free_indices.back();
          free_indices.pop_back();
          return &nodes[index];
        } else {
          return nullptr;
        }
      }

      void free(Node *node) noexcept {
        auto const index = node - nodes.data();
        free_indices.emplace_back(index);
      }

      bool owns(Node *node) const noexcept {
        return node >= nodes.data() && node < nodes.data() + nodes.size();
      }

      std::size_t size() const noexcept { return nodes.size(); }

    private:
      std::vector<Node> nodes;
      std::vector<std::ptrdiff_t> free_indices;
    };

    std::vector<Segment> _segments;
    std::size_t _capacity;
  };

  void free_internal_nodes(Node *root) noexcept {
    if (root->payload.index() == 0) {
      auto const children = std::get<0>(root->payload);
      _node_pool.free(root);
      for (auto const child : children) {
        if (child != nullptr) {
          free_internal_nodes(child);
        }
      }
    }
  }

  // TODO: handle exceptions here. for now just marking as noexcept so that
  // exceptions instantly kill the app
  Node *build_internal_node(std::span<Node *> leaf_nodes) noexcept {
    assert(leaf_nodes.size() > 1);
    auto const node = _node_pool.alloc();
    node->bounds = leaf_nodes[0]->bounds;
    for (auto it = leaf_nodes.begin() + 1; it != leaf_nodes.end(); ++it) {
      node->bounds = merge(node->bounds, (*it)->bounds);
    }
    auto const means = 0.5f * (node->bounds.min + node->bounds.max);
    auto const ranges = node->bounds.max - node->bounds.min;
    auto const axis_indices = [&]() {
      auto retval = std::array{0, 1, 2};
      auto const bubble = [&](auto const index) {
        if (ranges[retval[index]] < ranges[retval[index + 1]]) {
          std::swap(retval[index], retval[index + 1]);
        }
      };
      bubble(0);
      bubble(1);
      bubble(0);
      return retval;
    }();
    for (auto const axis_index : axis_indices) {
      auto const partition_iterator =
          partition(leaf_nodes, axis_index, means[axis_index]);
      auto const partitions = std::array<std::span<Node *>, 2>{
          std::span{leaf_nodes.begin(), partition_iterator},
          std::span{partition_iterator, leaf_nodes.end()}};
      if (!partitions[0].empty() && !partitions[1].empty()) {
        auto children = std::array<Node *, 2>{};
        for (auto i = 0; i < 2; ++i) {
          children[i] = partitions[i].size() == 1
                            ? partitions[i].front()
                            : build_internal_node(partitions[i]);
        }
        node->payload = children;
        return node;
      }
    }
    // if we got here partitioning failed and all leaf nodes' centroids coincide
    // so we do a bottom-up approach merging the smallest leaves first
    std::sort(leaf_nodes.begin(), leaf_nodes.end(), [](Node *a, Node *b) {
      return volume(a->bounds) < volume(b->bounds);
    });
    auto left_node = leaf_nodes[0];
    auto right_iterator = leaf_nodes.begin() + 1;
    for (;;) {
      auto const right_node = *right_iterator;
      if (++right_iterator == leaf_nodes.end()) {
        node->payload = std::array<Node *, 2>{left_node, right_node};
        return node;
      } else {
        auto const parent_node = _node_pool.alloc();
        parent_node->bounds = merge(left_node->bounds, right_node->bounds);
        parent_node->payload = std::array<Node *, 2>{left_node, right_node};
        left_node = parent_node;
      }
    }
  }

  auto partition(std::span<Node *> leaf_nodes, int axis_index,
                 float separating_position) noexcept {
    assert(!leaf_nodes.empty());
    auto position = [=](Node *node) {
      return 0.5f *
             (node->bounds.min[axis_index] + node->bounds.max[axis_index]);
    };
    auto unpartitioned_begin = leaf_nodes.begin();
    auto unpartitioned_end = leaf_nodes.end();
    do {
      auto &unpartitioned_leaf_node = *unpartitioned_begin;
      if (position(unpartitioned_leaf_node) < separating_position) {
        ++unpartitioned_begin;
      } else {
        --unpartitioned_end;
        std::swap(unpartitioned_leaf_node, *unpartitioned_end);
      }
    } while (unpartitioned_begin != unpartitioned_end);
    for (auto it = leaf_nodes.begin(); it != unpartitioned_begin; ++it) {
      assert(position(*it) < separating_position);
    }
    for (auto it = unpartitioned_begin; it != leaf_nodes.end(); ++it) {
      assert(position(*it) >= separating_position);
    }
    return unpartitioned_begin;
  }

  template <typename F>
  void for_each_overlapping_leaf_pair(Node *root, F &&f) noexcept(
      noexcept(f(std::declval<LeafPayload>(), std::declval<LeafPayload>()))) {
    assert(root != nullptr);
    if (root->payload.index() == 0) {
      auto const &children = std::get<0>(root->payload);
      for_each_overlapping_leaf_pair(children[0], std::forward<F>(f));
      for_each_overlapping_leaf_pair(children[1], std::forward<F>(f));
      for_each_overlapping_leaf_pair(children[0], children[1],
                                     std::forward<F>(f));
    }
  }

  template <typename F>
  void for_each_overlapping_leaf_pair(Node *left, Node *right, F &&f) noexcept(
      noexcept(f(std::declval<LeafPayload>(), std::declval<LeafPayload>()))) {
    if (overlaps(left->bounds, right->bounds)) {
      if (left->payload.index() == 0) {
        // left is internal
        // check left's children for overlap
        auto const &left_children = std::get<0>(left->payload);
        if (right->payload.index() == 0) {
          // right is internal
          // check right's children for overlap
          auto const &right_children = std::get<0>(right->payload);
          for (auto const left_child : left_children) {
            for (auto const right_child : right_children) {
              for_each_overlapping_leaf_pair(left_child, right_child,
                                             std::forward<F>(f));
            }
          }
        } else {
          // right is leaf
          for (auto const left_child : left_children) {
            for_each_overlapping_leaf_pair(left_child, right,
                                           std::forward<F>(f));
          }
        }
      } else {
        // left is leaf
        if (right->payload.index() == 0) {
          // right is internal
          auto const &right_children = std::get<0>(right->payload);
          for (auto const right_child : right_children) {
            for_each_overlapping_leaf_pair(left, right_child,
                                           std::forward<F>(f));
          }
        } else {
          // right is leaf
          f(std::get<1>(left->payload), std::get<1>(right->payload));
        }
      }
    }
  }

  Node_pool _node_pool;
  Node *_root_node{};
  ankerl::unordered_dense::set<Node *> _leaf_nodes;
  std::vector<Node *> _flattened_leaf_nodes;
};

using Bounding_box_hierarchy_leaf_payload =
    std::variant<Particle_reference, Static_rigid_body_reference>;

struct Particle {
  Bounding_box_hierarchy<Bounding_box_hierarchy_leaf_payload>::Node
      *bounding_box_hierarchy_leaf;
  Particle_motion_callback *motion_callback;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Vec3f previous_position;
  math::Vec3f current_position;
  math::Vec3f velocity;
  float mass;
  float mass_inverse;
  float radius;
  float static_friction_coefficient;
  float dynamic_friction_coefficient;
  float restitution_coefficient;
};

struct Static_rigid_body {
  Bounding_box_hierarchy<Bounding_box_hierarchy_leaf_payload>::Node
      *bounding_box_hierarchy_leaf;
  std::uint64_t collision_flags;
  std::uint64_t collision_mask;
  math::Mat3x4f transform;
  math::Mat3x4f transform_inverse;
  Shape *shape;
  float static_friction_coefficient;
  float dynamic_friction_coefficient;
  float restitution_coefficient;
};

struct Particle_particle_contact {
  std::array<Particle *, 2> particles;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};

struct Particle_static_rigid_body_contact {
  Particle *particle;
  Static_rigid_body *static_rigid_body;
  math::Vec3f normal;
  float normal_force_lagrange;
  float tangent_force_lagrange;
  float separating_velocity;
};
} // namespace

class Space::Impl {
public:
  explicit Impl(Space_create_info const &create_info)
      : _gravitational_acceleration{create_info.gravitational_acceleration} {}

  Particle_reference create_particle(Particle_create_info const &create_info) {
    auto const bounding_box =
        Bounding_box{create_info.position - math::Vec3f{create_info.radius,
                                                        create_info.radius,
                                                        create_info.radius},
                     create_info.position + math::Vec3f{create_info.radius,
                                                        create_info.radius,
                                                        create_info.radius}};
    Particle_reference const reference{_next_particle_reference_value};
    Particle const value{
        .bounding_box_hierarchy_leaf =
            _bounding_box_hierarchy.create_leaf(bounding_box, reference),
        .motion_callback = create_info.motion_callback,
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .previous_position = create_info.position,
        .current_position = create_info.position,
        .velocity = create_info.velocity,
        .mass = create_info.mass,
        .mass_inverse = 1.0f / create_info.mass,
        .radius = create_info.radius,
        .static_friction_coefficient = create_info.static_friction_coefficient,
        .dynamic_friction_coefficient =
            create_info.dynamic_friction_coefficient,
        .restitution_coefficient = create_info.restitution_coefficient};
    try {
      _particles.emplace(reference, value);
    } catch (...) {
      _bounding_box_hierarchy.destroy_leaf(value.bounding_box_hierarchy_leaf);
      throw;
    }
    ++_next_particle_reference_value;
    return reference;
  }

  void destroy_particle(Particle_reference reference) {
    auto const it = _particles.find(reference);
    _bounding_box_hierarchy.destroy_leaf(
        it->second.bounding_box_hierarchy_leaf);
    _particles.erase(it);
  }

  Static_rigid_body_reference
  create_static_rigid_body(Static_rigid_body_create_info const &create_info) {
    auto const transform = math::make_rigid_transform_mat3x4(
        create_info.position, create_info.orientation);
    auto const transform_inverse = math::rigid_inverse(transform);
    auto const bounding_box = create_info.shape->get_bounds(transform);
    Static_rigid_body_reference const reference{
        _next_static_rigid_body_reference_value};
    Static_rigid_body const value{
        .bounding_box_hierarchy_leaf =
            _bounding_box_hierarchy.create_leaf(bounding_box, reference),
        .collision_flags = create_info.collision_flags,
        .collision_mask = create_info.collision_mask,
        .transform = transform,
        .transform_inverse = transform_inverse,
        .shape = create_info.shape,
        .static_friction_coefficient = create_info.static_friction_coefficient,
        .dynamic_friction_coefficient =
            create_info.dynamic_friction_coefficient,
        .restitution_coefficient = create_info.restitution_coefficient};
    try {
      _static_rigid_bodies.emplace(reference, value);
    } catch (...) {
      _bounding_box_hierarchy.destroy_leaf(value.bounding_box_hierarchy_leaf);
      throw;
    }
    ++_next_static_rigid_body_reference_value;
    return reference;
  }

  void destroy_static_rigid_body(Static_rigid_body_reference reference) {
    auto const it = _static_rigid_bodies.find(reference);
    _bounding_box_hierarchy.destroy_leaf(
        it->second.bounding_box_hierarchy_leaf);
    _static_rigid_bodies.erase(it);
  }

  void simulate(Space_simulate_info const &simulate_info) {
    auto const h = simulate_info.delta_time / simulate_info.substep_count;
    auto const h_inv = 1.0f / h;
    find_contacts(simulate_info.delta_time);
    for (auto i = 0; i < simulate_info.substep_count; ++i) {
      for (auto &element : _particles) {
        auto &particle = element.second;
        particle.previous_position = particle.current_position;
        particle.velocity += h * _gravitational_acceleration;
        particle.current_position += h * particle.velocity;
      }
      solve_particle_particle_contact_positions();
      solve_particle_static_rigid_body_contact_positions();
      for (auto &element : _particles) {
        auto &particle = element.second;
        particle.velocity =
            h_inv * (particle.current_position - particle.previous_position);
      }
      solve_particle_particle_contact_velocities(h);
      solve_particle_static_rigid_body_contact_velocities(h);
    }
    for (auto &[reference, value] : _particles) {
      if (value.motion_callback != nullptr) {
        value.motion_callback->on_particle_motion(
            {reference, value.current_position, value.velocity});
      }
    }
  }

private:
  void find_contacts(float delta_time) {
    auto const safety_factor = 2.0f;
    auto const gravity_term =
        math::length(_gravitational_acceleration) * delta_time * delta_time;
    for (auto const &element : _particles) {
      auto const &particle = element.second;
      auto const radius =
          particle.radius +
          safety_factor * delta_time * math::length(particle.velocity) +
          gravity_term;
      auto const half_extents = math::Vec3f{radius, radius, radius};
      particle.bounding_box_hierarchy_leaf->bounds = {
          particle.current_position - half_extents,
          particle.current_position + half_extents};
    }
    _particle_particle_contacts.clear();
    _particle_static_rigid_body_contacts.clear();
    _bounding_box_hierarchy.build();
    _bounding_box_hierarchy.for_each_overlapping_leaf_pair(
        [this](Bounding_box_hierarchy_leaf_payload const &first_reference,
               Bounding_box_hierarchy_leaf_payload const &second_reference) {
          if (first_reference.index() == 0) {
            if (second_reference.index() == 0) {
              _particle_particle_contacts.emplace_back(
                  Particle_particle_contact{
                      .particles =
                          {&_particles.at(std::get<0>(first_reference)),
                           &_particles.at(std::get<0>(second_reference))},
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f});
            } else {
              _particle_static_rigid_body_contacts.emplace_back(
                  Particle_static_rigid_body_contact{
                      .particle = &_particles.at(std::get<0>(first_reference)),
                      .static_rigid_body = &_static_rigid_bodies.at(
                          std::get<1>(second_reference)),
                      .normal = math::Vec3f::zero(),
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f,
                      .separating_velocity = 0.0f});
            }
          } else {
            if (second_reference.index() == 0) {
              _particle_static_rigid_body_contacts.emplace_back(
                  Particle_static_rigid_body_contact{
                      .particle = &_particles.at(std::get<0>(second_reference)),
                      .static_rigid_body = &_static_rigid_bodies.at(
                          std::get<1>(first_reference)),
                      .normal = math::Vec3f::zero(),
                      .normal_force_lagrange = 0.0f,
                      .tangent_force_lagrange = 0.0f,
                      .separating_velocity = 0.0f});
            } else {
              // ignore static rigid bodies colliding with each other
            }
          }
        });
  }

  void solve_particle_particle_contact_positions() {
    for (auto &contact : _particle_particle_contacts) {
      auto const particle_a = contact.particles[0];
      auto const particle_b = contact.particles[1];
      auto const displacement =
          particle_a->current_position - particle_b->current_position;
      auto const distance2 = math::length2(displacement);
      auto const contact_distance = particle_a->radius + particle_b->radius;
      auto const contact_distance2 = contact_distance * contact_distance;
      if (distance2 < contact_distance2) {
        auto const [normal, separation] = [&]() {
          if (distance2 == 0.0f) {
            // particles coincide, pick arbitrary contact normal
            math::Vec3f const contact_normal{1.0f, 0.0f, 0.0f};
            auto const separation = -contact_distance;
            return std::tuple{contact_normal, separation};
          } else {
            auto const distance = std::sqrt(distance2);
            auto const normal = displacement / distance;
            auto const separation = distance - contact_distance;
            return std::tuple{normal, separation};
          }
        }();
        auto const delta_normal_force_lagrange =
            -separation / (particle_a->mass_inverse + particle_b->mass_inverse);
        auto const relative_motion =
            (particle_a->current_position - particle_a->previous_position) -
            (particle_b->current_position - particle_b->previous_position);
        auto const relative_tangential_motion =
            relative_motion - math::dot(relative_motion, normal) * normal;
        auto const delta_tangent_force_lagrange =
            math::length(relative_tangential_motion) /
            (particle_a->mass_inverse + particle_b->mass_inverse);
        contact.normal = normal;
        contact.normal_force_lagrange = delta_normal_force_lagrange;
        contact.tangent_force_lagrange = delta_tangent_force_lagrange;
        contact.separating_velocity =
            math::dot(particle_a->velocity - particle_b->velocity, normal);
        auto const normal_positional_impulse =
            delta_normal_force_lagrange * normal;
        particle_a->current_position +=
            normal_positional_impulse * particle_a->mass_inverse;
        particle_b->current_position -=
            normal_positional_impulse * particle_b->mass_inverse;
        auto const static_friction_coefficient =
            0.5f * (particle_a->static_friction_coefficient +
                    particle_b->static_friction_coefficient);
        if (contact.tangent_force_lagrange <
            static_friction_coefficient * contact.normal_force_lagrange) {
          auto const tangent_positional_impulse =
              -relative_tangential_motion /
              (particle_a->mass_inverse + particle_b->mass_inverse);
          particle_a->current_position +=
              tangent_positional_impulse * particle_a->mass_inverse;
          particle_b->current_position -=
              tangent_positional_impulse * particle_b->mass_inverse;
        }
      } else {
        contact.normal_force_lagrange = 0.0f;
        contact.tangent_force_lagrange = 0.0f;
      }
    }
  }

  void solve_particle_static_rigid_body_contact_positions() {
    for (auto &contact : _particle_static_rigid_body_contacts) {
      auto const particle = contact.particle;
      auto const static_rigid_body = contact.static_rigid_body;
      if (auto const contact_geometry =
              static_rigid_body->shape->collide_particle(
                  static_rigid_body->transform,
                  static_rigid_body->transform_inverse,
                  particle->current_position, particle->radius)) {
        auto const delta_normal_force_lagrange =
            -contact_geometry->separation * particle->mass;
        auto const relative_motion =
            particle->current_position - particle->previous_position;
        auto const relative_tangential_motion =
            relative_motion -
            math::dot(relative_motion, contact_geometry->normal) *
                contact_geometry->normal;
        auto const delta_tangent_force_lagrange =
            math::length(relative_tangential_motion) * particle->mass;
        contact.normal = contact_geometry->normal;
        contact.normal_force_lagrange = delta_normal_force_lagrange;
        contact.tangent_force_lagrange = delta_tangent_force_lagrange;
        contact.separating_velocity =
            math::dot(contact.particle->velocity, contact.normal);
        particle->current_position +=
            -contact_geometry->separation * contact_geometry->normal;
        auto const static_friction_coefficient =
            0.5f * (particle->static_friction_coefficient +
                    static_rigid_body->static_friction_coefficient);
        if (contact.tangent_force_lagrange <
            static_friction_coefficient * contact.normal_force_lagrange) {
          particle->current_position -= relative_tangential_motion;
        }
      } else {
        contact.normal_force_lagrange = 0.0f;
        contact.tangent_force_lagrange = 0.0f;
      }
    }
  }

  void solve_particle_particle_contact_velocities(float h) {
    auto const restitution_threshold =
        2.0f * math::length(_gravitational_acceleration) * h;
    for (auto &contact : _particle_particle_contacts) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v =
            contact.particles[0]->velocity - contact.particles[1]->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f * (contact.particles[0]->dynamic_friction_coefficient +
                      contact.particles[1]->dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(dynamic_friction_coefficient *
                           contact.normal_force_lagrange / h,
                       v_t_length);
          auto const friction_velocity_impulse =
              friction_delta_velocity / (contact.particles[0]->mass_inverse +
                                         contact.particles[1]->mass_inverse);
          contact.particles[0]->velocity +=
              friction_velocity_impulse * contact.particles[0]->mass_inverse;
          contact.particles[1]->velocity -=
              friction_velocity_impulse * contact.particles[1]->mass_inverse;
        }
        auto const restitution_coefficient =
            std::abs(v_n) > restitution_threshold
                ? 0.5f * (contact.particles[0]->restitution_coefficient +
                          contact.particles[1]->restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.normal * (-v_n + std::max(-restitution_coefficient *
                                                  contact.separating_velocity,
                                              0.0f));
        auto const restitution_velocity_impulse =
            restitution_delta_velocity / (contact.particles[0]->mass_inverse +
                                          contact.particles[1]->mass_inverse);
        contact.particles[0]->velocity +=
            restitution_velocity_impulse * contact.particles[0]->mass_inverse;
        contact.particles[1]->velocity -=
            restitution_velocity_impulse * contact.particles[1]->mass_inverse;
      }
    }
  }

  void solve_particle_static_rigid_body_contact_velocities(float h) {
    auto const restitution_threshold =
        2.0f * math::length(_gravitational_acceleration) * h;
    for (auto &contact : _particle_static_rigid_body_contacts) {
      if (contact.normal_force_lagrange != 0.0f) {
        auto const v = contact.particle->velocity;
        auto const v_n = math::dot(v, contact.normal);
        auto const v_t = v - contact.normal * v_n;
        auto const v_t_length2 = math::length2(v_t);
        if (v_t_length2 != 0.0f) {
          auto const v_t_length = std::sqrt(v_t_length2);
          auto const dynamic_friction_coefficient =
              0.5f * (contact.particle->dynamic_friction_coefficient +
                      contact.static_rigid_body->dynamic_friction_coefficient);
          auto const friction_delta_velocity =
              -v_t / v_t_length *
              std::min(dynamic_friction_coefficient *
                           contact.normal_force_lagrange / h,
                       v_t_length);
          contact.particle->velocity += friction_delta_velocity;
        }
        auto const restitution_coefficient =
            std::abs(v_n) > restitution_threshold
                ? 0.5f * (contact.particle->restitution_coefficient +
                          contact.static_rigid_body->restitution_coefficient)
                : 0.0f;
        auto const restitution_delta_velocity =
            contact.normal * (-v_n + std::max(-restitution_coefficient *
                                                  contact.separating_velocity,
                                              0.0f));
        contact.particle->velocity += restitution_delta_velocity;
      }
    }
  }

  Bounding_box_hierarchy<Bounding_box_hierarchy_leaf_payload>
      _bounding_box_hierarchy;
  ankerl::unordered_dense::map<Particle_reference, Particle> _particles;
  ankerl::unordered_dense::map<Static_rigid_body_reference, Static_rigid_body>
      _static_rigid_bodies;
  std::vector<Particle_particle_contact> _particle_particle_contacts;
  std::vector<Particle_static_rigid_body_contact>
      _particle_static_rigid_body_contacts;
  std::uint64_t _next_particle_reference_value{};
  std::uint64_t _next_static_rigid_body_reference_value{};
  math::Vec3f _gravitational_acceleration;
};

Space::Space(Space_create_info const &create_info)
    : _impl{std::make_unique<Impl>(create_info)} {}

Space::~Space() {}

Particle_reference
Space::create_particle(Particle_create_info const &create_info) {
  return _impl->create_particle(create_info);
}

void Space::destroy_particle(Particle_reference particle) {
  _impl->destroy_particle(particle);
}

Static_rigid_body_reference Space::create_static_rigid_body(
    Static_rigid_body_create_info const &create_info) {
  return _impl->create_static_rigid_body(create_info);
}

void Space::destroy_static_rigid_body(
    Static_rigid_body_reference static_rigid_body) {
  _impl->destroy_static_rigid_body(static_rigid_body);
}

void Space::simulate(Space_simulate_info const &simulate_info) {
  _impl->simulate(simulate_info);
}
} // namespace physics
} // namespace marlon