#ifndef MARLON_UTIL_STACK_H
#define MARLON_UTIL_STACK_H

#include <cstdlib>

#include <new>

#include "capacity_error.h"

namespace marlon {
namespace util {
template <typename T> class Stack {
public:
  explicit Stack(std::size_t capacity) {
    _begin = static_cast<T *>(std::malloc(capacity * sizeof(T)));
    if (_begin) {
      _stack_end = _begin;
      _buffer_end = _begin + capacity;
    } else {
      throw std::bad_alloc{};
    }
  }

  ~Stack() {
    while (!empty()) {
      pop_back();
    }
    free(_begin);
  }

  T const &back() const noexcept { return *(_stack_end - 1); }

  T &back() noexcept { return *(_stack_end - 1); }

  void push_back(T const &object) {
    if (_stack_end != _buffer_end) {
      new (_stack_end++) T(object);
    } else {
      throw Capacity_error{};
    }
  }

  void pop_back() noexcept { (--_stack_end)->~T(); }

  std::size_t size() const noexcept { return _stack_end - _begin; }

  std::size_t capacity() const noexcept { return _buffer_end - _begin; }

  bool empty() const noexcept { return size() == 0; }

private:
  T *_begin;
  T *_stack_end;
  T *_buffer_end;
};
} // namespace util
} // namespace marlon

#endif