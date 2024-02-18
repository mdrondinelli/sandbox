#ifndef MARLON_UTIL_SET_H
#define MARLON_UTIL_SET_H

#include <cassert>

#include <array>
#include <bit>
#include <utility>

#include "array.h"
#include "equal.h"
#include "hash.h"
#include "memory.h"

namespace marlon {
namespace util {
template <typename T, typename Hash = Hash<T>, typename Equal = Equal<T>>
class Set {
  struct Node;

  struct Bucket {
    Node *node{};
  };

  struct Node {
    Node *prev;
    Node *next;
    std::size_t hash;
    alignas(alignof(T)) std::array<std::byte, sizeof(T)> storage;

    T &value() noexcept { return *reinterpret_cast<T *>(&storage); }
  };

  static constexpr auto _alignment = std::max(alignof(Bucket), alignof(Node));

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
  memory_requirement(std::size_t max_node_count,
                     std::size_t max_bucket_count) noexcept {
    assert(std::has_one_bit(max_bucket_count));
    return Stack_allocator<_alignment>::memory_requirement({
        Array<Bucket>::memory_requirement(max_bucket_count),
        Pool_allocator<sizeof(Node)>::memory_requirement(max_node_count),
    });
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_node_count) noexcept {
    return memory_requirement(max_node_count, max_node_count);
  }

  constexpr Set() noexcept = default;

  explicit Set(Block block, std::size_t max_node_count) noexcept
      : Set{block, max_node_count, max_node_count} {}

  explicit Set(Block block, std::size_t max_node_count,
               std::size_t max_bucket_count) noexcept
      : Set{block.begin, max_node_count, max_bucket_count} {}

  explicit Set(void *block_begin, std::size_t max_node_count) noexcept
      : Set{block_begin, max_node_count, max_node_count} {}

  explicit Set(void *block_begin, std::size_t max_node_count,
               std::size_t max_bucket_count) noexcept {
    assert(std::has_one_bit(max_bucket_count));
    auto allocator = Stack_allocator<_alignment>{make_block(
        block_begin, memory_requirement(max_bucket_count, max_node_count))};
    _buckets = make_array<Bucket>(allocator, max_bucket_count).second;
    _buckets.resize(std::min(max_bucket_count, std::size_t{16}));
    _nodes =
        make_pool_allocator<sizeof(Node)>(allocator, max_node_count).second;
  }

  ~Set() {
    auto node = _head;
    while (node) {
      node->value().~T();
      _nodes.free(make_block(node, sizeof(Node)));
      node = node->next;
    }
  }

  Iterator begin() noexcept { return Iterator{_head}; }

  Const_iterator begin() const noexcept { return Const_iterator{_head}; }

  Const_iterator cbegin() const noexcept { return Const_iterator{_head}; }

  Iterator end() noexcept { return Iterator{nullptr}; }

  Const_iterator end() const noexcept { return Const_iterator{nullptr}; }

  Const_iterator cend() const noexcept { return Const_iterator{nullptr}; }

  std::size_t size() const noexcept { return _size; }

  void clear() noexcept {
    for (auto &bucket : _buckets) {
      bucket.node = nullptr;
    }
    auto node = _head;
    _head = nullptr;
    while (node) {
      node->value().~T();
      _nodes.free(make_block(node, sizeof(Node)));
      node = node->next;
    }
    _size = 0;
  }

  template <typename K> std::pair<Iterator, bool> insert(K &&x) {
    auto const hash = Hash{}(x);
    auto const index = hash & (_buckets.size() - 1);
    auto &bucket = _buckets[index];
    if (bucket.node == nullptr) {
      auto const block = [&]() {
        try {
          return _nodes.alloc(sizeof(Node));
        } catch (...) {
          throw Capacity_error{};
        }
      }();
      auto const node = new (block.begin) Node;
      node->prev = nullptr;
      node->next = _head;
      node->hash = hash;
      try {
        new (&node->storage) T(std::forward<K>(x));
      } catch (...) {
        _nodes.free(block);
        throw;
      }
      if (_head != nullptr) {
        _head->prev = node;
      }
      _head = node;
      bucket.node = node;
      if (++_size > _buckets.size() * _max_load_factor) {
        rehash(0);
      }
      return std::pair{Iterator{node}, true};
    } else {
      auto it = bucket.node;
      for (;;) {
        if (it->hash == hash) {
          if (Equal{}(it->value(), x)) {
            return std::pair{Iterator{it}, false};
          } else if (it->next == nullptr) {
            auto const block = [&]() {
              try {
                return _nodes.alloc(sizeof(Node));
              } catch (...) {
                throw Capacity_error{};
              }
            }();
            auto const node = new (block.begin) Node;
            node->prev = it;
            node->next = nullptr;
            node->hash = hash;
            try {
              new (&node->storage) T(std::forward<K>(x));
            } catch (...) {
              _nodes.free(block);
              throw;
            }
            it->next = node;
            if (++_size > _buckets.size() * _max_load_factor) {
              rehash(0);
            }
            return std::pair{Iterator{node}, true};
          } else {
            it = it->next;
          }
        } else if ((it->hash & (_buckets.size() - 1)) == index) {
          if (it->next == nullptr) {
            auto const block = [&]() {
              try {
                return _nodes.alloc(sizeof(Node));
              } catch (...) {
                throw Capacity_error{};
              }
            }();
            auto const node = new (block.begin) Node;
            node->prev = it;
            node->next = nullptr;
            node->hash = hash;
            try {
              new (&node->storage) T(std::forward<K>(x));
            } catch (...) {
              _nodes.free(block);
              throw;
            }
            it->next = node;
            if (++_size > _buckets.size() * _max_load_factor) {
              rehash(0);
            }
            return std::pair{Iterator{node}, true};
          } else {
            it = it->next;
          }
        } else {
          auto const block = [&]() {
            try {
              return _nodes.alloc(sizeof(Node));
            } catch (...) {
              throw Capacity_error{};
            }
          }();
          auto const node = new (block.begin) Node;
          node->prev = it->prev;
          node->next = it;
          node->hash = hash;
          try {
            new (&node->storage) T(std::forward<K>(x));
          } catch (...) {
            _nodes.free(block);
            throw;
          }
          it->prev->next = node;
          it->prev = node;
          if (++_size > _buckets.size() * _max_load_factor) {
            rehash(0);
          }
          return std::pair{Iterator{node}, true};
        }
      }
    }
  }

  template <typename... Args>
  std::pair<Iterator, bool> emplace(Args &&...args) {
    auto const block = [&]() {
      try {
        return _nodes.alloc(sizeof(Node));
      } catch (...) {
        throw Capacity_error{};
      }
    }();
    auto const node = new (block.begin) Node;
    try {
      new (&node->storage) T(std::forward<Args>(args)...);
    } catch (...) {
      _nodes.free(block);
      throw;
    }
    auto const hash = Hash{}(node->value());
    auto const index = hash & (_buckets.size() - 1);
    auto &bucket = _buckets[index];
    if (bucket.node == nullptr) {
      node->prev = nullptr;
      node->next = _head;
      node->hash = hash;
      if (_head != nullptr) {
        _head->prev = node;
      }
      _head = node;
      bucket.node = node;
      if (++_size > _buckets.size() * _max_load_factor) {
        rehash(0);
      }
      return std::pair{Iterator{node}, true};
    } else {
      // check if there's an equal element. If not, emplace
      auto it = bucket.node;
      for (;;) {
        if (it->hash == hash) {
          if (Equal{}(it->value(), node->value())) {
            node->value().~T();
            _nodes.free(block);
            return std::pair{Iterator{it}, false};
          } else if (it->next == nullptr) {
            node->prev = it;
            node->next = nullptr;
            node->hash = hash;
            it->next = node;
            if (++_size > _buckets.size() * _max_load_factor) {
              rehash(0);
            }
            return std::pair{Iterator{node}, true};
          } else {
            it = it->next;
          }
        } else if ((it->hash & (_buckets.size() - 1)) == index) {
          if (it->next == nullptr) {
            node->prev = it;
            node->next = nullptr;
            node->hash = hash;
            it->next = node;
            if (++_size > _buckets.size() * _max_load_factor) {
              rehash(0);
            }
            return std::pair{Iterator{node}, true};
          } else {
            it = it->next;
          }
        } else {
          node->prev = it->prev;
          node->next = it;
          node->hash = hash;
          it->prev->next = node;
          it->prev = node;
          if (++_size > _buckets.size() * _max_load_factor) {
            rehash(0);
          }
          return std::pair{Iterator{node}, true};
        }
      }
    }
  }

  Iterator erase(Iterator pos) noexcept { return erase(Const_iterator{pos}); }

  Iterator erase(Const_iterator pos) noexcept {
    pos._node->value().~T();
    if (pos._node->prev) {
      pos._node->prev->next = pos._node->next;
    }
    if (pos._node->next) {
      pos._node->next->prev = pos._node->prev;
    }
    auto result = Iterator{pos._node->next};
    _nodes.free(make_block(pos._node, sizeof(Node)));
    --_size;
    return result;
  }

  template <typename K> std::size_t erase(K const &x) noexcept {
    auto const pos = find(x);
    if (pos != end()) {
      erase(pos);
      return 1;
    } else {
      return 0;
    }
  }

  template <typename K> Iterator find(K const &x) noexcept {
    auto const hash = Hash{}(x);
    auto const index = hash & (_buckets.size() - 1);
    auto const &bucket = _buckets[index];
    auto it = bucket.node;
    for (;;) {
      if (it == nullptr) {
        return end();
      } else if (it->hash == hash) {
        if (Equal{}(it->value(), x)) {
          return Iterator{it};
        } else {
          it = it->next;
        }
      } else if ((it->hash & (_buckets.size() - 1)) == index) {
        it = it->next;
      } else {
        return end();
      }
    }
  }

  template <typename K> Const_iterator find(K const &x) const noexcept {
    auto const hash = Hash{}(x);
    auto const index = hash & (_buckets.size() - 1);
    auto const &bucket = _buckets[index];
    auto it = bucket.node;
    for (;;) {
      if (it == nullptr) {
        return end();
      } else if (it->hash == hash) {
        if (Equal{}(it->value(), x)) {
          return Iterator{it};
        } else {
          it = it->next;
        }
      } else if ((it->hash & (_buckets.size() - 1)) == index) {
        it = it->next;
      } else {
        return end();
      }
    }
  }

  std::size_t bucket_count() const noexcept { return _buckets.size(); }

  std::size_t max_bucket_count() const noexcept { return _buckets.capacity(); }

  float load_factor() const noexcept {
    return static_cast<float>(_size) / static_cast<float>(_buckets.size());
  }

  float max_load_factor() const noexcept { return _max_load_factor; }

  void max_load_factor(float ml) noexcept { _max_load_factor = ml; }

  void rehash(std::size_t count) noexcept {
    auto const n = std::min(
        std::bit_ceil(std::max(count, static_cast<std::size_t>(std::ceil(
                                          _size * _max_load_factor)))),
        _buckets.capacity());
    if (_buckets.size() == n) {
      return;
    }
    _buckets.resize(n);
    for (auto i = std::size_t{}; i != n; ++i) {
      _buckets[i].node = nullptr;
    }
    auto node = _head;
    _head = nullptr;
    while (node != nullptr) {
      auto const next = node->next;
      auto const index = node->hash & (n - 1);
      auto &bucket = _buckets[index];
      if (bucket.node == nullptr) {
        node->prev = nullptr;
        node->next = _head;
        if (_head != nullptr) {
          _head->prev = node;
        }
        _head = node;
      } else if (bucket.node == _head) {
        node->prev = nullptr;
        node->next = _head;
        _head->prev = node;
        _head = node;
      } else {
        node->prev = bucket.node->prev;
        node->next = bucket.node;
        bucket.node->prev->next = node;
        bucket.node->prev = node;
      }
      bucket.node = node;
      node = next;
    }
  }

private:
  Array<Bucket> _buckets;
  Pool_allocator<sizeof(Node)> _nodes;
  Node *_head{};
  std::size_t _size{};
  float _max_load_factor{1.0f};
};

template <typename T, typename Hash = Hash<T>, typename Equal = Equal<T>,
          typename Allocator>
std::pair<Block, Set<T, Hash, Equal>> make_set(Allocator &allocator,
                                               std::size_t max_node_count,
                                               std::size_t max_bucket_count) {
  auto const block = allocator.alloc(Set<T, Hash, Equal>::memory_requirement(
      max_node_count, max_bucket_count));
  return {block, Set<T, Hash, Equal>{block, max_node_count, max_bucket_count}};
}

template <typename T, typename Hash = Hash<T>, typename Equal = Equal<T>,
          typename Allocator>
std::pair<Block, Set<T, Hash, Equal>> make_set(Allocator &allocator,
                                               std::size_t max_node_count) {
  return make_set<T, Hash, Equal, Allocator>(allocator, max_node_count,
                                             max_node_count);
}
} // namespace util
} // namespace marlon

#endif