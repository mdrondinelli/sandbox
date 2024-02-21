#ifndef MARLON_UTIL_STACK_H
#define MARLON_UTIL_STACK_H

#include "capacity_error.h"
#include "lifetime_box.h"
#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Array {
public:
  using Const_iterator = T const *;
  using Iterator = T *;

  static constexpr std::size_t
  memory_requirement(std::size_t max_size) noexcept {
    return sizeof(T) * max_size;
  }

  constexpr Array() noexcept
      : _begin{nullptr}, _stack_end{nullptr}, _buffer_end{nullptr} {}

  explicit Array(Block block, std::size_t max_size) noexcept
      : Array{block.begin, max_size} {}

  explicit Array(void *block_begin, std::size_t max_size) noexcept
      : _begin{static_cast<T *>(block_begin)}, _stack_end{_begin},
        _buffer_end{_begin + max_size} {}

  Array(Array<T> &&other)
      : _begin{std::exchange(other._begin, nullptr)},
        _stack_end{std::exchange(other._stack_end, nullptr)},
        _buffer_end{std::exchange(other._buffer_end, nullptr)} {}

  Array &operator=(Array<T> &&other) {
    auto temp = Array<T>{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Array() { clear(); }

  T const &operator[](std::size_t index) const noexcept {
    return _begin[index];
  }

  T &operator[](std::size_t index) noexcept { return _begin[index]; }

  T const &front() const noexcept { return *_begin; }

  T &front() noexcept { return *_begin; }

  T const &back() const noexcept { return *(_stack_end - 1); }

  T &back() noexcept { return *(_stack_end - 1); }

  T const *data() const noexcept { return _begin; }

  T *data() noexcept { return _begin; }

  Const_iterator cbegin() const noexcept { return _begin; }

  Const_iterator begin() const noexcept { return _begin; }

  Iterator begin() noexcept { return _begin; }

  Const_iterator cend() const noexcept { return _stack_end; }

  Const_iterator end() const noexcept { return _stack_end; }

  Iterator end() noexcept { return _stack_end; }

  bool empty() const noexcept { return size() == 0; }

  std::size_t size() const noexcept { return _stack_end - _begin; }

  std::size_t max_size() const noexcept { return _buffer_end - _begin; }

  std::size_t capacity() const noexcept { return max_size(); }

  void clear() {
    auto const begin = _begin;
    auto const end = _stack_end;
    for (auto it = begin; it != end; ++it) {
      it->~T();
    }
    _stack_end = _begin;
  }

  void push_back(T const &object) {
    if (_stack_end != _buffer_end) {
      new (_stack_end++) T(object);
    } else {
      throw Capacity_error{};
    }
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (_stack_end != _buffer_end) {
      return *(new (_stack_end++) T(std::forward<Args>(args)...));
    } else {
      throw Capacity_error{};
    }
  }

  void pop_back() noexcept { (--_stack_end)->~T(); }

  void resize(std::size_t count) {
    auto const stack_end = _stack_end;
    auto const new_stack_end = _begin + count;
    if (new_stack_end > _buffer_end) {
      throw Capacity_error{};
    }
    if (new_stack_end < stack_end) {
      auto it = new_stack_end;
      do {
        it->~T();
        ++it;
      } while (it != stack_end);
      _stack_end = new_stack_end;
    } else if (stack_end < new_stack_end) {
      auto it = stack_end;
      do {
        new (it) T();
        ++it;
      } while (it != new_stack_end);
      _stack_end = new_stack_end;
    }
  }

private:
  void swap(Array<T> &other) noexcept {
    std::swap(_begin, other._begin);
    std::swap(_stack_end, other._stack_end);
    std::swap(_buffer_end, other._buffer_end);
  }

  T *_begin;
  T *_stack_end;
  T *_buffer_end;
};

template <typename T, typename Allocator>
std::pair<Block, Array<T>> make_array(Allocator &allocator,
                                      std::size_t max_size) {
  auto const block = allocator.alloc(Array<T>::memory_requirement(max_size));
  return {block, Array<T>{block, max_size}};
}

template <typename T, typename Allocator = Polymorphic_allocator>
class Dynamic_array {
public:
  using Iterator = Array<T>::Iterator;
  using Const_iterator = Array<T>::Const_iterator;

  Dynamic_array() : _allocator{} { _impl.construct(); }

  explicit Dynamic_array(const Allocator &allocator) : _allocator{allocator} {
    _impl.construct();
  }

  ~Dynamic_array() {
    if (_impl->max_size() != 0) {
      auto const block =
          make_block(_impl->data(), _impl->data() + _impl->max_size());
      _impl.destruct();
      _allocator.free(block);
    }
  }

  T const &operator[](std::size_t index) const noexcept {
    return (*_impl)[index];
  }

  T &operator[](std::size_t index) noexcept { return (*_impl)[index]; }

  T const &front() const noexcept { return _impl->front(); }

  T &front() noexcept { return _impl.front(); }

  T const &back() const noexcept { return _impl->back(); }

  T &back() noexcept { return _impl.back(); }

  T const *data() const noexcept { return _impl->data(); }

  T *data() noexcept { return _impl->data(); }

  Const_iterator cbegin() const noexcept { return _impl->cbegin(); }

  Const_iterator begin() const noexcept { return _impl->begin(); }

  Iterator begin() noexcept { return _impl->begin(); }

  Const_iterator cend() const noexcept { return _impl->cend(); }

  Const_iterator end() const noexcept { return _impl->end(); }

  Iterator end() noexcept { return _impl->end(); }

  bool empty() const noexcept { return _impl->empty(); }

  std::size_t size() const noexcept { return _impl->size(); }

  std::size_t max_size() const noexcept {
    return static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max());
  }

  void reserve(std::size_t new_cap) {
    if (new_cap > _impl->capacity()) {
      auto temp = make_array<T>(_allocator, new_cap).second;
      for (auto &object : *_impl) {
        temp.emplace_back(std::move(object));
      }
      auto const block =
          make_block(_impl->data(), _impl->data() + _impl->max_size());
      *_impl = std::move(temp);
      _allocator.free(block);
    }
  }

  std::size_t capacity() const noexcept { return _impl->capacity(); }

  void clear() { _impl->clear(); }

  void push_back(T const &object) {
    if (size() == capacity()) {
      reserve(size() * 2);
    }
    _impl->push_back(object);
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (size() == capacity()) {
      reserve(size() != 0 ? size() * 2 : 1);
    }
    return _impl->emplace_back(std::forward<Args>(args)...);
  }

  void pop_back() noexcept { _impl->pop_back(); }

  void resize(std::size_t count) {
    if (capacity() < count) {
      auto new_cap = capacity();
      while (new_cap < count) {
        new_cap *= 2;
      }
      reserve(new_cap);
    }
    _impl->resize(count);
  }

private:
  Allocator _allocator;
  Lifetime_box<Array<T>> _impl;
};
} // namespace util
} // namespace marlon

#endif