#ifndef MARLON_UTIL_CONTIGUOUS_STORAGE_H
#define MARLON_UTIL_CONTIGUOUS_STORAGE_H

#include <vector>

#include "capacity_error.h"

namespace marlon {
namespace util {
template <typename T> class Contiguous_storage {
public:
  explicit Contiguous_storage(std::size_t capacity) {
    _vector.reserve(capacity);
  }

  explicit Contiguous_storage(std::size_t capacity, std::size_t size) {
    _vector.reserve(capacity);
    _vector.resize(size);
  }

  T const &operator[](std::size_t index) const noexcept {
    return _vector[index];
  }

  T &operator[](std::size_t index) noexcept { return _vector[index]; }

  T const *data() const noexcept { return _vector.data(); }

  T *data() noexcept { return _vector.data(); }

  T const *cbegin() const noexcept { return data(); }

  T const *begin() const noexcept { return data(); }

  T *begin() noexcept { return data(); }

  T const *cend() const noexcept { return data() + size(); }

  T const *end() const noexcept { return data() + size(); }

  T *end() noexcept { return data() + size(); }

  std::size_t size() const noexcept { return _vector.size(); }

  std::size_t capacity() const noexcept { return _vector.capacity(); }

  void clear() noexcept { _vector.clear(); }

  void push_back(T const &value) {
    if (size() < capacity()) {
      _vector.push_back(value);
    } else {
      throw Capacity_error{};
    }
  }

  void push_back(T &&value) {
    if (size() < capacity()) {
      _vector.push_back(std::move(value));
    } else {
      throw Capacity_error{};
    }
  }

  void push_back_n(std::size_t n, T const &value) {
    for (auto i = std::size_t{}; i != n; ++i) {
      push_back(value);
    }
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (size() < capacity()) {
      return _vector.emplace_back(std::forward<Args>(args)...);
    } else {
      throw Capacity_error{};
    }
  }

  void pop_back() { _vector.pop_back(); }

private:
  std::vector<T> _vector;
};
} // namespace util
} // namespace marlon

#endif