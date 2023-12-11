#ifndef MARLON_UTIL_MAP_H
#define MARLON_UTIL_MAP_H

#include <cassert>
#include <cstddef>

#include "memory.h"
#include "stack.h"

namespace marlon {
namespace util {
template <typename K, typename V> class Map {
public:

  class Node {
  public:
    // Node(Node *next, )

    K const &key() const noexcept { return _key; }

    V const &value() const noexcept { return _value; }

    V &value() noexcept { return _value; }

  private:
    Node *_next;
    K _key;
    V _value;
  };

  static constexpr std::size_t
  memory_requirement(std::size_t max_buckets, std::size_t max_nodes) noexcept {
    return Stack_allocator<alignof(Node)>::memory_requirement(
        {Stack<Node *>::memory_requirement(max_buckets),
         Stack_allocator<alignof(Node)>::memory_requirement(sizeof(Node) *
                                                            max_nodes)});
  }

  explicit Map(Block block, std::size_t max_buckets,
               std::size_t max_nodes) noexcept
      : _impl{[&]() {
          auto allocator = Stack_allocator<alignof(Node)>{block};
          auto const buckets_block =
              allocator.alloc(Stack<Node *>::memory_requirement(max_buckets));
          auto const node_allocator_block = allocator.alloc(
              Stack_allocator<alignof(Node)>::memory_requirement(sizeof(Node) *
                                                                 max_nodes));
          return Impl{
              .buckets = Stack<Node *>{max_buckets},
              .node_allocator =
                  Node_allocator{Stack_allocator<alignof(Node)>{nodes_block}},
              .node_count = {}};
        }()} {
    assert((max_buckets & max_buckets - 1) == 0);
  }

  explicit Map(void *block_begin, std::size_t max_buckets,
               std::size_t max_nodes) noexcept
      : Map{make_block(block_begin, memory_requirement(max_buckets, max_nodes)),
            max_buckets, max_nodes} {}

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
          auto const temp = node;
          node = node->next;
          destroy_node(temp);
        } while (node);
        bucket = nullptr;
      }
    }
  }

  float load_factor() const noexcept {
    return static_cast<float>(_impl.node_count) /
           static_cast<float>(buckets.capacity());
  }

private:
  using Node_allocator =
      Free_list_allocator<Stack_allocator<alignof(Node)>, sizeof(Node)>;

  struct Impl {
    Stack<Node *> buckets;
    Node_allocator node_allocator;
    std::size_t node_count;
  };

  Node *create_node() {
    return new (_impl.node_allocator.alloc(sizeof(Node)).begin) Node;
  }

  void destroy_node(Node *node) noexcept {
    _impl.node_allocator.free(make_block(node, sizeof(Node)));
  }

  Impl _impl;
};
} // namespace util
} // namespace marlon

#endif