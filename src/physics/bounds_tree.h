#ifndef MARLON_PHYSICS_BOUNDS_TREE_H
#define MARLON_PHYSICS_BOUNDS_TREE_H

#include <array>
#include <span>
#include <utility>
#include <variant>

#include "bounds.h"

namespace marlon {
namespace physics {
// Payload must be nothrow copyable
template <typename Payload> class Bounds_tree {
public:
  struct Node {
    Node *parent;
    Bounds bounds;
    std::variant<std::array<Node *, 2>, Payload> payload;
  };

  Node *create_leaf(Bounds const &bounds, Payload const &payload) {
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
      noexcept(f(std::declval<Payload>(), std::declval<Payload>()))) {
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
      auto const partition_iterator =
          partition(leaf_nodes, axis_index, node_center[axis_index]);
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
} // namespace physics
} // namespace marlon

#endif