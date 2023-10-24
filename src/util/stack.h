#ifndef MARLON_UTIL_STACK_H
#define MARLON_UTIL_STACK_H

#include "capacity_error.h"
#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class Stack {
public:
  using Const_iterator = T const *;
  using Iterator = T *;

  static constexpr std::size_t
  memory_requirement(std::size_t capacity) noexcept {
    return sizeof(T) * capacity;
  }

  explicit Stack(Block block, std::size_t capacity) noexcept
      : Stack{block.begin, capacity} {}

  explicit Stack(void *block_begin, std::size_t capacity) noexcept
      : _begin{static_cast<T *>(block_begin)}, _stack_end{_begin},
        _buffer_end{_begin + capacity} {}

  ~Stack() { clear(); }

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

  std::size_t capacity() const noexcept { return _buffer_end - _begin; }

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

  // void push_back_n(T const &object, std::size_t n) {
  //   for (auto i = std::size_t{}; i != n; ++i) {
  //     push_back(object);
  //   }
  // }

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
  T *_begin;
  T *_stack_end;
  T *_buffer_end;
};
} // namespace util
} // namespace marlon

#endif