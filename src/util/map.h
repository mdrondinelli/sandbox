#ifndef MARLON_UTIL_MAP_H
#define MARLON_UTIL_MAP_H

#include "set.h"

namespace marlon {
namespace util {
template <typename K, typename V, typename KeyHash = Hash<K>,
          typename KeyEqual = Equal<K>>
class Map {
  struct PairHash {
    template <std::convertible_to<std::pair<K, V>> P>
    constexpr std::size_t operator()(P const &x) const noexcept {
      return KeyHash{}(x.first);
    }

    template <typename T>
    constexpr std::size_t operator()(T const &x) const noexcept {
      return KeyHash{}(x);
    }
  };

  struct PairEqual {
    template <std::convertible_to<std::pair<K, V>> P>
    constexpr bool operator()(std::pair<K, V> const &lhs,
                              P const &rhs) const noexcept {
      return KeyEqual{}(lhs.first, rhs.first);
    }

    template <typename T>
    constexpr bool operator()(std::pair<K, V> const &lhs,
                              T const &rhs) const noexcept {
      return KeyEqual{}(lhs.first, rhs);
    }
  };

  Set<std::pair<K, V>, PairHash, PairEqual> _impl;

public:
  using Iterator = decltype(_impl)::Iterator;
  using Const_iterator = decltype(_impl)::Const_iterator;

  static constexpr std::size_t
  memory_requirement(std::size_t max_node_count) noexcept {
    return memory_requirement(max_node_count, max_node_count);
  }

  static constexpr std::size_t
  memory_requirement(std::size_t max_node_count,
                     std::size_t max_bucket_count) noexcept {
    return decltype(_impl)::memory_requirement(max_node_count,
                                               max_bucket_count);
  }

  constexpr Map() noexcept = default;

  explicit Map(Block block, std::size_t max_node_count) noexcept
      : _impl{block, max_node_count} {}

  explicit Map(Block block, std::size_t max_node_count,
               std::size_t max_bucket_count)
      : _impl{block, max_node_count, max_bucket_count} {}

  explicit Map(void *block_begin, std::size_t max_node_count) noexcept
      : _impl{block_begin, max_node_count} {}

  explicit Map(void *block_begin, std::size_t max_node_count,
               std::size_t max_bucket_count) noexcept
      : _impl{block_begin, max_node_count, max_bucket_count} {}

  void const *data() const noexcept { return _impl.data(); }

  void *data() noexcept { return _impl.data(); }

  Iterator begin() noexcept { return _impl.begin(); }

  Const_iterator begin() const noexcept { return _impl.begin(); }

  Const_iterator cbegin() const noexcept { return _impl.cbegin(); }

  Iterator end() noexcept { return _impl.end(); }

  Const_iterator end() const noexcept { return _impl.end(); }

  Const_iterator cend() const noexcept { return _impl.cend(); }

  std::size_t size() const noexcept { return _impl.size(); }

  std::size_t max_size() const noexcept { return _impl.max_size(); }

  void clear() noexcept { _impl.clear(); }

  template <typename P> std::pair<Iterator, bool> insert(P &&x) {
    return _impl.insert(std::forward<P>(x));
  }

  template <typename... Args>
  std::pair<Iterator, bool> emplace(Args &&...args) {
    return _impl.emplace(std::forward<Args>(args)...);
  }

  Iterator erase(Iterator pos) noexcept { return _impl.erase(pos); }

  Iterator erase(Const_iterator pos) noexcept { return _impl.erase(pos); }

  template <typename T> std::size_t erase(T const &x) noexcept {
    return _impl.erase(x);
  }

  template <typename T> Iterator find(T const &x) noexcept {
    return _impl.find(x);
  }

  template <typename T> Const_iterator find(T const &x) const noexcept {
    return _impl.find(x);
  }

  template <typename T> V &at(T const &k) noexcept {
    return find(k)->second;
  }

  template <typename T> V const &at(T const &k) const noexcept {
    return find(k)->second;
  }

  std::size_t bucket_count() const noexcept { return _impl.bucket_count(); }

  std::size_t max_bucket_count() const noexcept {
    return _impl.max_bucket_count();
  }

  float load_factor() const noexcept { return _impl.load_factor(); }

  float max_load_factor() const noexcept { return _impl.max_load_factor(); }

  void max_load_factor(float ml) noexcept { _impl.max_load_factor(ml); }

  void rehash(std::size_t count) noexcept { _impl.rehash(count); }
};
} // namespace util
} // namespace marlon

#endif