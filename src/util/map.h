#ifndef MARLON_UTIL_MAP_H
#define MARLON_UTIL_MAP_H

#include "set.h"

namespace marlon {
namespace util {
template <typename K, typename V, typename KeyHash = Hash<K>, typename KeyEqual = Equal<K>> class Map {
  struct PairHash {
    template <std::convertible_to<std::pair<K, V>> P> constexpr std::size_t operator()(P const &x) const noexcept {
      return KeyHash{}(x.first);
    }

    template <typename T> constexpr std::size_t operator()(T const &x) const noexcept { return KeyHash{}(x); }
  };

  struct PairEqual {
    template <std::convertible_to<std::pair<K, V>> P>
    constexpr bool operator()(std::pair<K, V> const &lhs, P const &rhs) const noexcept {
      return KeyEqual{}(lhs.first, rhs.first);
    }

    template <typename T> constexpr bool operator()(std::pair<K, V> const &lhs, T const &rhs) const noexcept {
      return KeyEqual{}(lhs.first, rhs);
    }
  };

  Set<std::pair<K, V>, PairHash, PairEqual> _impl;

public:
  using Iterator = typename decltype(_impl)::Iterator;
  using Const_iterator = typename decltype(_impl)::Const_iterator;

  template <typename Allocator> static std::pair<Block, Map> make(Allocator &allocator, Size max_node_count) {
    return make(allocator, max_node_count, max_node_count);
  }

  template <typename Allocator>
  static std::pair<Block, Map> make(Allocator &allocator, Size max_node_count, Size max_bucket_count) {
    auto const block = allocator.alloc(memory_requirement(max_node_count, max_bucket_count));
    return {block, Map{block, max_node_count, max_bucket_count}};
  }

  static constexpr Size memory_requirement(Size max_node_count) noexcept {
    return memory_requirement(max_node_count, max_node_count);
  }

  static constexpr Size memory_requirement(Size max_node_count, Size max_bucket_count) noexcept {
    return decltype(_impl)::memory_requirement(max_node_count, max_bucket_count);
  }

  constexpr Map() noexcept = default;

  explicit Map(Block block, Size max_node_count) noexcept : _impl{block, max_node_count} {}

  explicit Map(Block block, Size max_node_count, Size max_bucket_count)
      : _impl{block, max_node_count, max_bucket_count} {}

  explicit Map(std::byte *block_begin, Size max_node_count) noexcept : _impl{block_begin, max_node_count} {}

  explicit Map(std::byte *block_begin, Size max_node_count, Size max_bucket_count) noexcept
      : _impl{block_begin, max_node_count, max_bucket_count} {}

  Const_block block() const noexcept { return _impl.block(); }

  Iterator begin() noexcept { return _impl.begin(); }

  Const_iterator begin() const noexcept { return _impl.begin(); }

  Const_iterator cbegin() const noexcept { return _impl.cbegin(); }

  Iterator end() noexcept { return _impl.end(); }

  Const_iterator end() const noexcept { return _impl.end(); }

  Const_iterator cend() const noexcept { return _impl.cend(); }

  Size size() const noexcept { return _impl.size(); }

  Size max_size() const noexcept { return _impl.max_size(); }

  void clear() noexcept { _impl.clear(); }

  template <typename P> std::pair<Iterator, bool> insert(P &&x) { return _impl.insert(std::forward<P>(x)); }

  template <typename... Args> std::pair<Iterator, bool> emplace(Args &&...args) {
    return _impl.emplace(std::forward<Args>(args)...);
  }

  Iterator erase(Iterator pos) noexcept { return _impl.erase(pos); }

  Iterator erase(Const_iterator pos) noexcept { return _impl.erase(pos); }

  template <typename T> Size erase(T const &x) noexcept { return _impl.erase(x); }

  template <typename T> Iterator find(T const &x) noexcept { return _impl.find(x); }

  template <typename T> Const_iterator find(T const &x) const noexcept { return _impl.find(x); }

  template <typename T> V &at(T const &k) noexcept { return find(k)->second; }

  template <typename T> V const &at(T const &k) const noexcept { return find(k)->second; }

  Size bucket_count() const noexcept { return _impl.bucket_count(); }

  Size max_bucket_count() const noexcept { return _impl.max_bucket_count(); }

  float load_factor() const noexcept { return _impl.load_factor(); }

  float max_load_factor() const noexcept { return _impl.max_load_factor(); }

  void max_load_factor(float ml) noexcept { _impl.max_load_factor(ml); }

  void rehash(Size count) noexcept { _impl.rehash(count); }
};

template <typename K,
          typename V,
          typename KeyHash = Hash<K>,
          typename KeyEqual = Equal<K>,
          typename Allocator = System_allocator>
class Allocating_map : Allocator {
public:
  using Iterator = typename Map<K, V, KeyHash, KeyEqual>::Iterator;
  using Const_iterator = typename Map<K, V, KeyHash, KeyEqual>::Const_iterator;

  Allocating_map() { _impl.construct(); }

  explicit Allocating_map(Allocator const &allocator) : Allocator{allocator} { _impl.construct(); }

  Allocating_map(Allocating_map &&other) noexcept : Allocator{std::move(static_cast<Allocator &>(other))} {
    _impl.construct(std::move(*other._impl));
  }

  Allocating_map &operator=(Allocating_map &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Allocating_map() {
    if (_impl->max_size() != 0 || _impl->max_bucket_count() != 0) {
      auto const block = _impl->block();
      _impl.destruct();
      Allocator::free(block);
    }
  }

  Const_block block() const noexcept { return _impl->block(); }

  // void const *data() const noexcept { return _impl->data(); }

  // void *data() noexcept { return _impl->data(); }

  Iterator begin() noexcept { return _impl->begin(); }

  Const_iterator begin() const noexcept { return _impl->begin(); }

  Const_iterator cbegin() const noexcept { return _impl->cbegin(); }

  Iterator end() noexcept { return _impl->end(); }

  Const_iterator end() const noexcept { return _impl->end(); }

  Const_iterator cend() const noexcept { return _impl->cend(); }

  Size size() const noexcept { return _impl->size(); }

  Size max_size() const noexcept { return std::numeric_limits<std::ptrdiff_t>::max(); }

  void clear() noexcept { _impl->clear(); }

  template <typename P> std::pair<Iterator, bool> insert(P &&x) {
    prepare_for_new_element();
    return _impl->insert(std::forward<P>(x));
  }

  template <typename... Args> std::pair<Iterator, bool> emplace(Args &&...args) {
    prepare_for_new_element();
    return _impl->emplace(std::forward<Args>(args)...);
  }

  Iterator erase(Iterator pos) noexcept { return _impl->erase(pos); }

  Iterator erase(Const_iterator pos) noexcept { return _impl->erase(pos); }

  template <typename T> Size erase(T const &x) noexcept { return _impl->erase(x); }

  template <typename T> Iterator find(T const &x) noexcept { return _impl->find(x); }

  template <typename T> Const_iterator find(T const &x) const noexcept { return _impl->find(x); }

  template <typename T> V &at(T const &k) noexcept { return find(k)->second; }

  template <typename T> V const &at(T const &k) const noexcept { return find(k)->second; }

  Size bucket_count() const noexcept { return _impl->bucket_count(); }

  Size max_bucket_count() const noexcept {
    return std::numeric_limits<std::ptrdiff_t>::max() ^ (std::numeric_limits<std::ptrdiff_t>::max() >> 1);
  }

  float load_factor() const noexcept { return _impl->load_factor(); }

  float max_load_factor() const noexcept { return _impl->max_load_factor(); }

  void max_load_factor(float ml) noexcept {}

  void rehash(Size count) noexcept {
    auto const max_bucket_count = static_cast<Size>(std::bit_ceil(static_cast<std::size_t>(
        std::max(count, static_cast<Size>(std::ceil(size() / static_cast<double>(max_load_factor())))))));
    auto const max_node_count = static_cast<Size>(max_bucket_count * static_cast<double>(max_load_factor()));
    if (max_bucket_count > _impl->max_bucket_count() || max_node_count > _impl->max_size()) {
      auto temp =
          Map<K, V, KeyHash, KeyEqual>::make(static_cast<Allocator &>(*this), max_node_count, max_bucket_count).second;
      temp.rehash(max_bucket_count);
      for (auto &object : *_impl) {
        temp.emplace(std::move(object));
      }
      if (auto const block = _impl->block(); block.size() != 0) {
        *_impl = std::move(temp);
        Allocator::free(block);
      } else {
        *_impl = std::move(temp);
      }
    } else {
      _impl->rehash(max_bucket_count);
    }
  }

  void reserve(Size count) {
    rehash(static_cast<Size>(
        std::bit_ceil(static_cast<std::size_t>(std::ceil(count / static_cast<double>(max_load_factor()))))));
  }

private:
  void swap(Allocating_map &other) noexcept {
    using std::swap;
    swap(static_cast<Allocator &>(*this), static_cast<Allocator &>(other));
    swap(*_impl, *other._impl);
  }

  void prepare_for_new_element() {
    if (size() == _impl->max_size()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
  }

  Lifetime_box<Map<K, V, KeyHash, KeyEqual>> _impl;
};
} // namespace util
} // namespace marlon

#endif