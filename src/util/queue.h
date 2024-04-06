#ifndef MARLON_UTIL_QUEUE_H
#define MARLON_UTIL_QUEUE_H

#include <cstddef>

#include <limits>
#include <span>
#include <utility>

#include "capacity_error.h"
#include "lifetime_box.h"
#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Queue {
public:
  class Iterator {
  public:
    T &operator*() const noexcept { return _slots[_index]; }

    T *operator->() const noexcept { return &_slots[_index]; }

    Iterator &operator++() noexcept {
      _index = (_index + 1) % _slots.size();
      ++_offset;
      return *this;
    }

    Iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Iterator &operator--() noexcept {
      _index = (_index + _slots.size() - 1) % _slots.size();
      --_offset;
      return *this;
    }

    Iterator operator--(int) noexcept {
      auto temp{*this};
      --*this;
      return temp;
    }

    friend bool operator==(Iterator const &lhs, Iterator const &rhs) noexcept {
      return lhs._offset == rhs._offset;
    }

  private:
    friend class Const_iterator;
    friend class Queue;

    explicit Iterator(std::span<T> slots,
                      std::size_t index,
                      std::size_t offset) noexcept
        : _slots{slots}, _index{index}, _offset{offset} {}

    std::span<T> _slots;
    std::size_t _index;
    std::size_t _offset;
  };

  class Const_iterator {
  public:
    Const_iterator(Iterator it) noexcept
        : _slots{it._slots}, _index{it._index}, _offset{it._offset} {}

    T const &operator*() const noexcept { return _slots[_index]; }

    T const *operator->() const noexcept { return &_slots[_index]; }

    Const_iterator &operator++() noexcept {
      _index = (_index + 1) % _slots.size();
      ++_offset;
      return *this;
    }

    Const_iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Const_iterator &operator--() noexcept {
      _index = (_index + _slots.size() - 1) % _slots.size();
      --_offset;
      return *this;
    }

    Const_iterator operator--(int) noexcept {
      auto temp{*this};
      --*this;
      return temp;
    }

    friend bool operator==(Const_iterator const &lhs,
                           Const_iterator const &rhs) noexcept {
      return lhs._offset == rhs._offset;
    }

  private:
    friend class Queue;

    std::span<T const> _slots;
    std::size_t _index;
    std::size_t _offset;
  };

  static constexpr std::size_t
  memory_requirement(std::size_t max_size) noexcept {
    return sizeof(T) * max_size;
  }

  constexpr Queue() noexcept = default;

  explicit Queue(Block block, std::size_t max_size) noexcept
      : Queue{block.begin, max_size} {}

  explicit Queue(void *block, std::size_t max_size) noexcept
      : _slots{static_cast<T *>(block), max_size} {}

  Queue(Queue<T> &&other) noexcept
      : _slots{std::exchange(other._slots, std::span<T>{})},
        _size{std::exchange(other._size, std::size_t{})},
        _head{std::exchange(other._head, std::size_t{})},
        _tail{std::exchange(other._tail, std::size_t{})} {}

  Queue &operator=(Queue<T> &&other) noexcept {
    auto temp = Queue<T>{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Queue() { clear(); }

  T const &front() const noexcept { return _slots[_head]; }

  T &front() noexcept { return _slots[_head]; }

  T const &back() const noexcept {
    return _slots[(_tail + _slots.size() - 1) % _slots.size()];
  }

  T &back() noexcept {
    return _slots[(_tail + _slots.size() - 1) % _slots.size()];
  }

  void const *data() const noexcept { return _slots.data(); }

  void *data() noexcept { return _slots.data(); }

  Const_iterator cbegin() const noexcept {
    return Const_iterator{_slots, _head, 0};
  }

  Const_iterator begin() const noexcept { return cbegin(); }

  Iterator begin() noexcept { return Iterator{_slots, _head, 0}; }

  Const_iterator cend() const noexcept {
    return Const_iterator{_slots, _tail, _size};
  }

  Const_iterator end() const noexcept { return cend(); }

  Iterator end() noexcept { return Iterator{_slots, _tail, _size}; }

  bool empty() const noexcept { return size() == 0; }

  std::size_t size() const noexcept { return _size; }

  std::size_t max_size() const noexcept { return _slots.size(); }

  std::size_t capacity() const noexcept { return max_size(); }

  void clear() noexcept {
    for (std::size_t i = 0; i != _size; ++i) {
      _slots[(_head + i) % _slots.size()].~T();
    }
    _size = {};
    _head = {};
    _tail = {};
  }

  void push_front(T const &object) {
    if (_size != _slots.size()) {
      auto const index = (_head + _slots.size() - 1) % _slots.size();
      new (&_slots[index]) T(object);
      ++_size;
      _head = index;
    } else {
      throw Capacity_error{};
    }
  }

  template <typename... Args> T &emplace_front(Args &&...args) {
    if (_size != _slots.size()) {
      auto const index = (_head + _slots.size() - 1) % _slots.size();
      auto &result = *new (&_slots[index]) T(std::forward<Args>(args)...);
      ++_size;
      _head = index;
      return result;
    } else {
      throw Capacity_error{};
    }
  }

  void pop_front() noexcept {
    _slots[_head].~T();
    --_size;
    _head = (_head + 1) % _slots.size();
  }

  void push_back(T const &object) {
    if (_size != _slots.size()) {
      new (&_slots[_tail]) T(object);
      ++_size;
      _tail = (_tail + 1) % _slots.size();
    } else {
      throw Capacity_error{};
    }
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (_size != _slots.size()) {
      auto &result = *new (&_slots[_tail]) T(std::forward<Args>(args)...);
      ++_size;
      _tail = (_tail + 1) % _slots.size();
      return result;
    } else {
      throw Capacity_error{};
    }
  }

  void pop_back() noexcept {
    auto const index = (_tail + _slots.size() - 1) % _slots.size();
    _slots[index].~T();
    --_size;
    _tail = index;
  }

private:
  void swap(Queue<T> &other) noexcept {
    std::swap(_slots, other._slots);
    std::swap(_size, other._size);
    std::swap(_head, other._head);
    std::swap(_tail, other._tail);
  }

  std::span<T> _slots;
  std::size_t _size{};
  std::size_t _head{};
  std::size_t _tail{};
};

template <typename T, typename Allocator>
std::pair<Block, Queue<T>> make_queue(Allocator &allocator,
                                      std::size_t max_size) {
  auto const block = allocator.alloc(Queue<T>::memory_requirement(max_size));
  return {block, Queue<T>{block, max_size}};
}

template <typename T, class Allocator = Polymorphic_allocator>
class Allocating_queue {
public:
  using Iterator = typename Queue<T>::Iterator;
  using Const_iterator = typename Queue<T>::Const_iterator;

  Allocating_queue() { _impl.construct(); }

  explicit Allocating_queue(Allocator const &allocator)
      : _allocator{allocator} {
    _impl.construct();
  }

  ~Allocating_queue() {
    if (_impl->data() != nullptr) {
      auto const block = make_block(
          _impl->data(), Queue<T>::memory_requirement(_impl->max_size()));
      _impl.destruct();
      _allocator.free(block);
    }
  }

  T const &front() const noexcept { return _impl->front(); }

  T &front() noexcept { return _impl->front(); }

  T const &back() const noexcept { return _impl->back(); }

  T &back() noexcept { return _impl->back(); }

  void const *data() const noexcept { return _impl->data(); }

  void *data() noexcept { return _impl->data(); }

  Const_iterator cbegin() const noexcept { return _impl->cbegin(); }

  Const_iterator begin() const noexcept { return _impl->begin(); }

  Iterator begin() noexcept { return _impl->begin(); }

  Const_iterator cend() const noexcept { return _impl->cend(); }

  Const_iterator end() const noexcept { return _impl->end(); }

  Iterator end() noexcept { return _impl->end(); }

  bool empty() const noexcept { return _impl->empty(); }

  std::size_t size() const noexcept { return _impl->size(); }

  std::size_t max_size() const noexcept {
    return return std::numeric_limits<std::size_t>::max();
  }

  std::size_t capacity() const noexcept { return _impl->capacity(); }

  void reserve(std::size_t capacity) {
    if (capacity > _impl->capacity()) {
      auto temp = make_queue<T>(_allocator, capacity).second;
      for (auto &object : *_impl) {
        temp.emplace_back(std::move(object));
      }
      if (_impl->data() != nullptr) {
        auto const block = make_block(
            _impl->data(), Queue<T>::memory_requirement(_impl->max_size()));
        *_impl = std::move(temp);
        _allocator.free(block);
      } else {
        *_impl = std::move(temp);
      }
    }
  }

  void clear() noexcept { _impl->clear(); }

  void push_front(T const &object) {
    prepare_for_new_element();
    _impl->push_front(object);
  }

  template <typename... Args> T &emplace_front(Args &&...args) {
    prepare_for_new_element();
    return _impl->emplace_front(std::forward<Args>(args)...);
  }

  void pop_front() noexcept { _impl->pop_front(); }

  void push_back(T const &object) {
    prepare_for_new_element();
    _impl->push_back(object);
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    prepare_for_new_element();
    return _impl->emplace_back(std::forward<Args>(args)...);
  }

  void pop_back() noexcept { _impl->pop_back(); }

private:
  void prepare_for_new_element() {
    if (size() == capacity()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
  }

  Allocator _allocator;
  Lifetime_box<Queue<T>> _impl;
};
} // namespace util
} // namespace marlon

#endif