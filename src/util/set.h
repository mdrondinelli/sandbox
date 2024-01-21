#ifndef MARLON_UTIL_SET_H
#define MARLON_UTIL_SET_H

#include <array>
#include <utility>

#include "equal.h"
#include "hash.h"
#include "memory.h"
#include "stack.h"

namespace marlon {
namespace util {
template <typename T, typename Hash = Hash<T>, typename Equal = Equal<T>>
class Set {
  struct Node;

  struct Bucket {
    Node *node;
  };

  struct Node {
    Node *prev;
    Node *next;
    alignas(alignof(T)) std::array<std::byte, sizeof(T)> storage;

    T &value() noexcept { return *reinterpret_cast<T *>(&storage); }
  };

public:
  class Iterator {
    friend class Set;
    friend class Const_iterator;

  public:
    T &operator*() const noexcept { return _node->value(); }

    T *operator->() const noexcept { return &_node->value(); }

    Iterator &operator++() noexcept {
      if (_node) {
        _node = _node->next;
      }
      return *this;
    }

    Iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Iterator &operator--() noexcept {
      if (_node) {
        _node = _node->prev;
      }
      return *this;
    }

    Iterator operator--(int) noexcept {
      auto temp{*this};
      --*temp;
      return temp;
    }

    friend bool operator==(Iterator lhs, Iterator rhs) noexcept {
      return lhs._node == rhs._node;
    }

  private:
    explicit Iterator(Node *node) noexcept : _node{node} {}

    Node *_node;
  };

  class Const_iterator {
    friend class Set;

  public:
    Const_iterator(Iterator it) noexcept : _node{it._node} {}

    T const &operator*() const noexcept { return _node->value(); }

    T const *operator->() const noexcept { return &_node->value(); }

    Const_iterator &operator++() noexcept {
      if (_node) {
        _node = _node->next;
      }
      return *this;
    }

    Const_iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Const_iterator &operator--() noexcept {
      if (_node) {
        _node = _node->prev;
      }
      return *this;
    }

    Const_iterator operator--(int) noexcept {
      auto temp{*this};
      --*this;
      return temp;
    }

    friend bool operator==(Const_iterator lhs, Const_iterator rhs) noexcept {
      return lhs._node == rhs._node;
    }

  private:
    explicit Const_iterator(Node *node) noexcept : _node{node} {}

    Node *_node;
  };

  static constexpr std::size_t
  memory_requirement(std::size_t bucket_count,
                     std::size_t node_count) noexcept {
    return Stack_allocator<alignof(Node)>::memory_requirement({
        Stack<Bucket>::memory_requirement(bucket_count),
        Stack_allocator<alignof(Node)>::memory_requirement(
            {node_count * sizeof(Node)}),
    });
  }

  constexpr Set() noexcept = default;

  explicit Set(Block block, std::size_t bucket_count,
               std::size_t node_count) noexcept
      : Set{block.begin, bucket_count, node_count} {}

  explicit Set(void *block_begin, std::size_t bucket_count,
               std::size_t node_count) noexcept {
    auto allocator = Stack_allocator<alignof(Node)>{
        make_block(block_begin, memory_requirement(bucket_count, node_count))};
    _buckets = Stack<Bucket>{
        allocator.alloc(Stack<Bucket>::memory_requirement(bucket_count))};
    _nodes = Free_list_allocator<Stack_allocator<alignof(Node)>, sizeof(Node),
                                 sizeof(Node)>{Stack_allocator<alignof(Node)>{
        allocator.alloc(node_count * sizeof(Node))}};
  }

  ~Set() { clear(); }

  Iterator begin() noexcept { return Iterator{_head}; }

  Const_iterator begin() const noexcept { return Const_iterator{_head}; }

  Const_iterator cbegin() const noexcept { return Const_iterator{_head}; }

  Iterator end() noexcept { return Iterator{nullptr}; }

  Const_iterator end() const noexcept { return Const_iterator{nullptr}; }

  Const_iterator cend() const noexcept { return Const_iterator{nullptr}; }

  void clear() noexcept {
    auto node = _head;
    _head = nullptr;
    while (node) {
      node->value().~T();
      node = node->next;
      _nodes.free(make_block(node, sizeof(Node)));
    }
  }

  // template <typename... Args>
  // std::pair<Iterator, bool> emplace(Args &&...args) {

  // }

private:
  Stack<Bucket> _buckets;
  Free_list_allocator<Stack_allocator<alignof(Node)>, sizeof(Node),
                      sizeof(Node)>
      _nodes;
  Node *_head{};
};
} // namespace util
} // namespace marlon

#endif