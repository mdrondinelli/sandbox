#ifndef MARLON_UTIL_MAP_H
#define MARLON_UTIL_MAP_H

#include <cassert>
#include <cstddef>

#include "stack.h"

namespace marlon {
namespace util {
template <typename K, typename V> class Map {
public:
  explicit Map(std::size_t max_nodes, std::size_t max_buckets)
      : _nodes{max_nodes}, _buckets{max_buckets}, _free_nodes{max_nodes} {
    assert((max_buckets & max_buckets - 1) == 0);
    _nodes.resize(max_nodes);
    _buckets.resize(max_buckets);
    for (auto &node : _nodes) {
      _free_nodes.emplace_back(&node);
    }
  }

  ~Map() {
    for (auto node : _buckets) {
      while (node) {
        reinterpret_cast<K *>(node->key.data())->~K();
        reinterpret_cast<V *>(node->value.data())->~V();
        node = node->next;
      }
    }
  }

  void clear() {
    for (auto &bucket : _buckets) {
      if (auto node = bucket) {
        do {
          reinterpret_cast<K *>(node->key.data())->~K();
          reinterpret_cast<V *>(node->value.data())->~V();
          node = node->next;
        } while (node);
        bucket = nullptr;
      }
    }
  }

private:
  struct Node {
    Node *next;
    alignas(K) std::array<std::byte, sizeof(K)> key;
    alignas(V) std::array<std::byte, sizeof(V)> value;
  };

  util::Stack<Node> _nodes;
  util::Stack<Node *> _buckets;
  util::Stack<Node *> _free_nodes;
};
} // namespace util
} // namespace marlon

#endif