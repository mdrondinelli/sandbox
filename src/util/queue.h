#ifndef MARLON_UTIL_QUEUE_H
#define MARLON_UTIL_QUEUE_H

#include <cstdlib>

#include <new>

#include "capacity_error.h"

namespace marlon {
namespace util {
template <typename T> class Queue {
public:
  explicit Queue(std::size_t capacity) {
    auto const buffer = static_cast<T *>(std::malloc(sizeof(T) * capacity));
    if (buffer) {
      _begin = buffer;
      _end = buffer + capacity;
      _front = _begin;
      _back = _end - 1;
      _size = 0;
    } else {
      throw std::bad_alloc{};
    }
  }

  ~Queue() {
    while (!empty()) {
      pop_front();
    }
    std::free(_begin);
  }

  // UB if queue is empty
  T const &front() const noexcept { return *_front; }

  // UB if queue is empty
  T &front() noexcept { return *_front; }

  // UB if queue is empty
  T const &back() const noexcept { return *_back; }

  // UB if queue is empty
  T &back() noexcept { return *_back; }

  // throws if queue is full
  void push_back(T const &object) {
    if (static_cast<std::ptrdiff_t>(_size) != _end - _begin) {
      ++_back;
      if (_back == _end) {
        _back = _begin;
      }
      new (_back) T(object);
      ++_size;
    } else {
      throw Capacity_error{};
    }
  }

  // UB if queue is empty
  void pop_front() noexcept {
    (_front)->~T();
    ++_front;
    if (_front == _end) {
      _front = _begin;
    }
    --_size;
  }

  std::size_t size() const noexcept { return _size; }

  bool empty() const noexcept { return _size == 0; }

private:
  T *_begin;
  T *_end;
  T *_front;
  T *_back;
  std::size_t _size;
};
} // namespace util
} // namespace marlon

#endif