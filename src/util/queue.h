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
    T &operator*() const noexcept {
      return _slots[_index];
    }

    T *operator->() const noexcept {
      return &_slots[_index];
    }

    Iterator &operator++() noexcept {
      _index = (_index + 1) & (_slots.size() - 1);
      ++_offset;
      return *this;
    }

    Iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Iterator &operator--() noexcept {
      _index = (_index + _slots.size() - 1) & (_slots.size() - 1);
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

    explicit Iterator(std::span<T> slots, Size index, Size offset) noexcept
        : _slots{slots}, _index{index}, _offset{offset} {}

    std::span<T> _slots;
    Size _index;
    Size _offset;
  };

  class Const_iterator {
  public:
    Const_iterator(Iterator it) noexcept
        : _slots{it._slots}, _index{it._index}, _offset{it._offset} {}

    T const &operator*() const noexcept {
      return _slots[_index];
    }

    T const *operator->() const noexcept {
      return &_slots[_index];
    }

    Const_iterator &operator++() noexcept {
      _index = (_index + 1) & (_slots.size() - 1);
      ++_offset;
      return *this;
    }

    Const_iterator operator++(int) noexcept {
      auto temp{*this};
      ++*this;
      return temp;
    }

    Const_iterator &operator--() noexcept {
      _index = (_index + _slots.size() - 1) & (_slots.size() - 1);
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
    Size _index;
    Size _offset;
  };

  template <typename Allocator>
  static std::pair<Block, Queue> make(Allocator &allocator, Size max_size) {
    auto const block = allocator.alloc(memory_requirement(max_size));
    return {block, Queue{block, max_size}};
  }

  static constexpr Size memory_requirement(Size max_size) noexcept {
    if (max_size != 0) {
      return static_cast<Size>(
          sizeof(T) * std::bit_ceil(static_cast<std::size_t>(max_size)));
    } else {
      return 0;
    }
  }

  constexpr Queue() noexcept = default;

  explicit Queue(Block block, Size max_size) noexcept
      : Queue{block.begin, max_size} {}

  explicit Queue(void *block, Size max_size) noexcept
      : _slots{static_cast<T *>(block),
               max_size != 0 ? std::bit_ceil(static_cast<std::size_t>(max_size))
                             : 0} {}

  Queue(Queue<T> &&other) noexcept
      : _slots{std::exchange(other._slots, std::span<T>{})},
        _size{std::exchange(other._size, Size{})},
        _head{std::exchange(other._head, Size{})},
        _tail{std::exchange(other._tail, Size{})} {}

  Queue &operator=(Queue<T> &&other) noexcept {
    auto temp = Queue<T>{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Queue() {
    clear();
  }

  Const_block block() const noexcept {
    return {reinterpret_cast<std::byte const *>(_slots.data()),
            static_cast<Size>(_slots.size_bytes())};
  }

  T const &front() const noexcept {
    return _slots[_head];
  }

  T &front() noexcept {
    return _slots[_head];
  }

  T const &back() const noexcept {
    return _slots[(_tail + _slots.size() - 1) & (_slots.size() - 1)];
  }

  T &back() noexcept {
    return _slots[(_tail + _slots.size() - 1) & (_slots.size() - 1)];
  }

  Const_iterator cbegin() const noexcept {
    return Const_iterator{_slots, _head, 0};
  }

  Const_iterator begin() const noexcept {
    return cbegin();
  }

  Iterator begin() noexcept {
    return Iterator{_slots, _head, 0};
  }

  Const_iterator cend() const noexcept {
    return Const_iterator{_slots, _tail, _size};
  }

  Const_iterator end() const noexcept {
    return cend();
  }

  Iterator end() noexcept {
    return Iterator{_slots, _tail, _size};
  }

  bool empty() const noexcept {
    return size() == 0;
  }

  Size size() const noexcept {
    return _size;
  }

  Size max_size() const noexcept {
    return _slots.size();
  }

  Size capacity() const noexcept {
    return max_size();
  }

  void clear() noexcept {
    for (Size i = 0; i != _size; ++i) {
      _slots[(_head + i) & (_slots.size() - 1)].~T();
    }
    _size = {};
    _head = {};
    _tail = {};
  }

  void push_front(T const &object) {
    if (_size != _slots.size()) {
      auto const index = (_head + _slots.size() - 1) & (_slots.size() - 1);
      new (&_slots[index]) T(object);
      ++_size;
      _head = index;
    } else {
      throw Capacity_error{};
    }
  }

  template <typename... Args> T &emplace_front(Args &&...args) {
    if (_size != static_cast<Size>(_slots.size())) {
      auto const index = (_head + _slots.size() - 1) & (_slots.size() - 1);
      auto &result = *new (&_slots[index]) T(std::forward<Args>(args)...);
      ++_size;
      _head = index;
      return result;
    } else {
      throw Capacity_error{"Capacity_error in Queue::emplace_front"};
    }
  }

  void pop_front() noexcept {
    _slots[_head].~T();
    --_size;
    _head = (_head + 1) & (_slots.size() - 1);
  }

  void push_back(T const &object) {
    if (_size != static_cast<Size>(_slots.size())) {
      new (&_slots[_tail]) T(object);
      ++_size;
      _tail = (_tail + 1) & (_slots.size() - 1);
    } else {
      throw Capacity_error{"Capacity_error in Queue::push_back"};
    }
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (_size != static_cast<Size>(_slots.size())) {
      auto &result = *new (&_slots[_tail]) T(std::forward<Args>(args)...);
      ++_size;
      _tail = (_tail + 1) & (_slots.size() - 1);
      return result;
    } else {
      throw Capacity_error{"Capacity_error in Queue::emplace_back"};
    }
  }

  void pop_back() noexcept {
    auto const index = (_tail + _slots.size() - 1) & (_slots.size() - 1);
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
  Size _size{};
  Size _head{};
  Size _tail{};
};

template <typename T, class Allocator = System_allocator>
class Allocating_queue : Allocator {
public:
  using Iterator = typename Queue<T>::Iterator;
  using Const_iterator = typename Queue<T>::Const_iterator;

  Allocating_queue() {
    _impl.construct();
  }

  explicit Allocating_queue(Allocator const &allocator)
      : Allocator{allocator} {
    _impl.construct();
  }

  ~Allocating_queue() {
    if (auto const block = _impl->block(); block.size() != 0) {
      // auto const block = make_block(
      //     _impl->data(), Queue<T>::memory_requirement(_impl->max_size()));
      _impl.destruct();
      Allocator::free(block);
    }
  }

  Const_block block() const noexcept {
    return _impl->block();
  }

  T const &front() const noexcept {
    return _impl->front();
  }

  T &front() noexcept {
    return _impl->front();
  }

  T const &back() const noexcept {
    return _impl->back();
  }

  T &back() noexcept {
    return _impl->back();
  }

  void const *data() const noexcept {
    return _impl->data();
  }

  void *data() noexcept {
    return _impl->data();
  }

  Const_iterator cbegin() const noexcept {
    return _impl->cbegin();
  }

  Const_iterator begin() const noexcept {
    return _impl->begin();
  }

  Iterator begin() noexcept {
    return _impl->begin();
  }

  Const_iterator cend() const noexcept {
    return _impl->cend();
  }

  Const_iterator end() const noexcept {
    return _impl->end();
  }

  Iterator end() noexcept {
    return _impl->end();
  }

  bool empty() const noexcept {
    return _impl->empty();
  }

  Size size() const noexcept {
    return _impl->size();
  }

  Size max_size() const noexcept {
    return std::numeric_limits<Size>::max();
  }

  Size capacity() const noexcept {
    return _impl->capacity();
  }

  void reserve(Size capacity) {
    if (capacity > _impl->capacity()) {
      auto temp =
          Queue<T>::make(static_cast<Allocator &>(*this), capacity).second;
      for (auto &object : *_impl) {
        temp.emplace_back(std::move(object));
      }
      if (auto const block = _impl->block(); block.size() != 0) {
        *_impl = std::move(temp);
        Allocator::free(block);
      } else {
        *_impl = std::move(temp);
      }
    }
  }

  void clear() noexcept {
    _impl->clear();
  }

  void push_front(T const &object) {
    prepare_for_new_element();
    _impl->push_front(object);
  }

  template <typename... Args> T &emplace_front(Args &&...args) {
    prepare_for_new_element();
    return _impl->emplace_front(std::forward<Args>(args)...);
  }

  void pop_front() noexcept {
    _impl->pop_front();
  }

  void push_back(T const &object) {
    prepare_for_new_element();
    _impl->push_back(object);
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    prepare_for_new_element();
    return _impl->emplace_back(std::forward<Args>(args)...);
  }

  void pop_back() noexcept {
    _impl->pop_back();
  }

private:
  void prepare_for_new_element() {
    if (size() == capacity()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
  }

  Lifetime_box<Queue<T>> _impl;
};
} // namespace util
} // namespace marlon

#endif