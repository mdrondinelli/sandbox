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
    // assert(std::has_single_bit(max_bucket_count));
    return Stack_allocator<_alignment>::memory_requirement({
        Array<Bucket>::memory_requirement(std::bit_ceil(max_bucket_count)),
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
    max_bucket_count = std::bit_ceil(max_bucket_count);
    auto allocator = Stack_allocator<_alignment>{make_block(
        block_begin, memory_requirement(max_node_count, max_bucket_count))};
    _buckets = make_array<Bucket>(allocator, max_bucket_count).second;
    _buckets.resize(1);
    _nodes =
        make_pool_allocator<sizeof(Node)>(allocator, max_node_count).second;
  }

  Set(Set &&other) noexcept
      : _buckets{std::move(other._buckets)}, _nodes{std::move(other._nodes)},
        _head{std::exchange(other._head, nullptr)},
        _size{std::exchange(other._size, 0)},
        _max_load_factor{other._max_load_factor} {}

  Set &operator=(Set &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Set() {
    auto node = _head;
    while (node) {
      node->value().~T();
      node = node->next;
    }
  }

  void const *data() const noexcept { return _buckets.data(); }

  void *data() noexcept { return _buckets.data(); }

  Iterator begin() noexcept { return Iterator{_head}; }

  Const_iterator begin() const noexcept { return Const_iterator{_head}; }

  Const_iterator cbegin() const noexcept { return Const_iterator{_head}; }

  Iterator end() noexcept { return Iterator{nullptr}; }

  Const_iterator end() const noexcept { return Const_iterator{nullptr}; }

  Const_iterator cend() const noexcept { return Const_iterator{nullptr}; }

  std::size_t size() const noexcept { return _size; }

  std::size_t max_size() const noexcept { return _nodes.max_blocks(); }

  void clear() noexcept {
    for (auto &bucket : _buckets) {
      bucket.node = nullptr;
    }
    auto node = _head;
    _head = nullptr;
    while (node) {
      node->value().~T();
      auto const next = node->next;
      _nodes.free(make_block(node, sizeof(Node)));
      node = next;
      --_size;
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
                                          _size / _max_load_factor)))),
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
  void swap(Set<T, Hash, Equal> &other) noexcept {
    std::swap(_buckets, other._buckets);
    std::swap(_nodes, other._nodes);
    std::swap(_head, other._head);
    std::swap(_size, other._size);
    std::swap(_max_load_factor, other._max_load_factor);
  }

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

template <typename T, typename Hash = Hash<T>, typename Equal = Equal<T>,
          typename Allocator = Polymorphic_allocator>
class Dynamic_set {
public:
  using Iterator = Set<T, Hash, Equal>::Iterator;
  using Const_iterator = Set<T, Hash, Equal>::Const_iterator;

  Dynamic_set() { _impl.construct(); }

  explicit Dynamic_set(Allocator const &allocator) : _allocator{allocator} {
    _impl.construct();
  }

  ~Dynamic_set() {
    if (_impl->max_size() != 0 || _impl->max_bucket_count() != 0) {
      auto const block = make_block(
          _impl->data(), Set<T, Hash, Equal>::memory_requirement(
                             _impl->max_size(), _impl->max_bucket_count()));
      _impl.destruct();
      _allocator.free(block);
    }
  }

  void const *data() const noexcept { return _impl->data(); }

  void *data() noexcept { return _impl->data(); }

  Iterator begin() noexcept { return _impl->begin(); }

  Const_iterator begin() const noexcept { return _impl->begin(); }

  Const_iterator cbegin() const noexcept { return _impl->cbegin(); }

  Iterator end() noexcept { return _impl->end(); }

  Const_iterator end() const noexcept { return _impl->end(); }

  Const_iterator cend() const noexcept { return _impl->cend(); }

  std::size_t size() const noexcept { return _impl->size(); }

  std::size_t max_size() const noexcept {
    return std::numeric_limits<std::ptrdiff_t>::max();
  }

  void clear() noexcept { _impl->clear(); }

  template <typename K> std::pair<Iterator, bool> insert(K &&x) {
    if (size() == _impl->max_size()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
    return _impl->insert(std::forward<K>(x));
  }

  template <typename... Args>
  std::pair<Iterator, bool> emplace(Args &&...args) {
    if (size() == _impl->max_size()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
    return _impl->emplace(std::forward<Args>(args)...);
  }

  Iterator erase(Iterator pos) noexcept { return _impl->erase(pos); }

  Iterator erase(Const_iterator pos) noexcept { return _impl->erase(pos); }

  template <typename K> std::size_t erase(K const &x) noexcept {
    return _impl->erase(x);
  }

  template <typename K> Iterator find(K const &x) noexcept {
    return _impl->find(x);
  }

  template <typename K> Const_iterator find(K const &x) const noexcept {
    return _impl->find(x);
  }

  std::size_t bucket_count() const noexcept { return _impl->bucket_count(); }

  std::size_t max_bucket_count() const noexcept {
    return std::numeric_limits<std::ptrdiff_t>::max() ^
           (std::numeric_limits<std::ptrdiff_t>::max() >> 1);
  }

  float load_factor() const noexcept { return _impl->load_factor(); }

  float max_load_factor() const noexcept { return _impl->max_load_factor(); }

  void max_load_factor(float ml) noexcept {}

  void rehash(std::size_t count) noexcept {
    auto const n = std::bit_ceil(std::max(
        count,
        static_cast<std::size_t>(std::ceil(size() / max_load_factor()))));
    if (n > _impl->max_bucket_count()) {
      auto temp =
          make_set<T, Hash, Equal>(
              _allocator, static_cast<std::size_t>(n * max_load_factor()), n)
              .second;
      temp.rehash(n);
      for (auto &object : *_impl) {
        temp.emplace(std::move(object));
      }
      if (_impl->max_bucket_count() > 0) {
        auto const block = make_block(
            _impl->data(), Set<T, Hash, Equal>::memory_requirement(
                               _impl->max_size(), _impl->max_bucket_count()));
        *_impl = std::move(temp);
        _allocator.free(block);
      } else {
        *_impl = std::move(temp);
      }
    } else {
      _impl->rehash(n);
    }
  }

  void reserve(std::size_t count) {
    rehash(std::bit_ceil(
        static_cast<std::size_t>(std::ceil(count / max_load_factor()))));
  }

private:
  Allocator _allocator;
  Lifetime_box<Set<T, Hash, Equal>> _impl;
};
} // namespace util
} // namespace marlon

#endif