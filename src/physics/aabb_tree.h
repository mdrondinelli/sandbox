#ifndef MARLON_PHYSICS_BOUNDS_TREE_H
#define MARLON_PHYSICS_BOUNDS_TREE_H

#include <array>
#include <span>
#include <utility>
#include <variant>

#include "aabb.h"

namespace marlon {
namespace physics {
// Payload must be nothrow copyable
// Payload destructor doesn't get called when node is destroyed
template <typename Payload> class Aabb_tree {
public:
  struct Node {
    Node *parent;
    Aabb bounds;
    std::variant<std::array<Node *, 2>, Payload> payload;
  };

  Node *create_leaf(Aabb const &bounds, Payload const &payload) {
    auto const node = _leaf_node_pool.alloc();
    try {
      _leaf_nodes.emplace(node);
    } catch (...) {
      _leaf_node_pool.free(node);
      throw;
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
    _leaf_node_pool.free(node);
  }

  void build() {
    if (_root_node != nullptr) {
      // free_internal_nodes(_root_node);
      _internal_node_pool.free_all();
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
      noexcept(f(std::declval<Payload>(), std::declval<Payload>()))) {
    if (_root_node != nullptr) {
      for_each_overlapping_leaf_pair(_root_node, std::forward<F>(f));
    }
  }

private:
  class Leaf_node_pool {
  public:
    Leaf_node_pool() {
      // approximately 4MB of leaf nodes
      auto const capacity = 4 * 1024 * 1024 / sizeof(Node);
      _nodes.resize(capacity);
      _free_indices.resize(capacity);
      for (auto i = std::ptrdiff_t{}; i != capacity; ++i) {
        _free_indices[i] = i;
      }
    }

    Leaf_node_pool(const Leaf_node_pool &other) = delete;

    Leaf_node_pool &operator=(const Leaf_node_pool &other) = delete;

    Node *alloc() {
      if (_free_indices.empty()) {
        throw std::runtime_error{"Out of space for leaf aabb tree nodes"};
      }
      auto const index = _free_indices.back();
      _free_indices.pop_back();
      return _nodes.data() + index;
    }

    void free(Node *node) noexcept {
      _free_indices.emplace_back(node - _nodes.data());
    }

  private:
    std::vector<Node> _nodes;
    std::vector<std::ptrdiff_t> _free_indices;
  };

  class Internal_node_pool {
  public:
    Internal_node_pool() : _nodes(4 * 1024 * 1024 / sizeof(Node)), _index{} {}

    Node *alloc() {
      if (static_cast<std::size_t>(_index) == _nodes.size()) {
        throw std::runtime_error{"Out of space for internal aabb tree nodes"};
      }
      return _nodes.data() + _index++;
    }

    void free_all() noexcept { _index = 0; }

  private:
    std::vector<Node> _nodes;
    std::ptrdiff_t _index;
  };

  // TODO: handle exceptions here. for now just marking as noexcept so that
  // exceptions instantly kill the app
  Node *build_internal_node(std::span<Node *> leaf_nodes) noexcept {
    assert(leaf_nodes.size() > 1);
    auto const node = _internal_node_pool.alloc();
    node->bounds = leaf_nodes[0]->bounds;
    for (auto it = leaf_nodes.begin() + 1; it != leaf_nodes.end(); ++it) {
      node->bounds = merge(node->bounds, (*it)->bounds);
    }
    auto const node_center = center(node->bounds);
    auto const node_extents = extents(node->bounds);
    auto const axis_indices = [&]() {
      auto retval = std::array{0, 1, 2};
      auto const bubble = [&](auto const index) {
        if (node_extents[retval[index]] < node_extents[retval[index + 1]]) {
          std::swap(retval[index], retval[index + 1]);
        }
      };
      bubble(0);
      bubble(1);
      bubble(0);
      return retval;
    }();
    for (auto const axis_index : axis_indices) {
      auto const split_position = node_center[axis_index];
      auto const partition_iterator =
          partition(leaf_nodes, axis_index, split_position);
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
        auto const parent_node = _internal_node_pool.alloc();
        parent_node->bounds = merge(left_node->bounds, right_node->bounds);
        parent_node->payload = std::array<Node *, 2>{left_node, right_node};
        left_node = parent_node;
      }
    }
  }

  auto partition(std::span<Node *> leaf_nodes, int axis_index,
                 float split_position) noexcept {
    assert(!leaf_nodes.empty());
    auto const double_split_position = 2.0f * split_position;
    auto double_position = [=](Node *node) {
      return (node->bounds.min[axis_index] + node->bounds.max[axis_index]);
    };
    auto unpartitioned_begin = leaf_nodes.begin();
    auto unpartitioned_end = leaf_nodes.end();
    do {
      auto &unpartitioned_leaf_node = *unpartitioned_begin;
      if (double_position(unpartitioned_leaf_node) < double_split_position) {
        ++unpartitioned_begin;
      } else {
        --unpartitioned_end;
        std::swap(unpartitioned_leaf_node, *unpartitioned_end);
      }
    } while (unpartitioned_begin != unpartitioned_end);
    return unpartitioned_begin;
  }

  template <typename F>
  void for_each_overlapping_leaf_pair(Node *root, F &&f) noexcept(
      noexcept(f(std::declval<Payload>(), std::declval<Payload>()))) {
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
      noexcept(f(std::declval<Payload>(), std::declval<Payload>()))) {
    if (overlaps(left->bounds, right->bounds)) {
      if (left->payload.index() == 0) {
        // left is internal
        auto const &left_children = std::get<0>(left->payload);
        if (right->payload.index() == 0) {
          // right is internal
          auto const &right_children = std::get<0>(right->payload);
          for_each_overlapping_leaf_pair(left_children[0], right_children[0],
                                         std::forward<F>(f));
          for_each_overlapping_leaf_pair(left_children[0], right_children[1],
                                         std::forward<F>(f));
          for_each_overlapping_leaf_pair(left_children[1], right_children[0],
                                         std::forward<F>(f));
          for_each_overlapping_leaf_pair(left_children[1], right_children[1],
                                         std::forward<F>(f));
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

  Leaf_node_pool _leaf_node_pool;
  Internal_node_pool _internal_node_pool;
  Node *_root_node{};
  ankerl::unordered_dense::set<Node *> _leaf_nodes;
  std::vector<Node *> _flattened_leaf_nodes;
};
} // namespace physics
} // namespace marlon

#endif