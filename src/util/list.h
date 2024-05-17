#ifndef MARLON_UTIL_STACK_H
#define MARLON_UTIL_STACK_H

#include <cstddef>

#include <utility>

#include "capacity_error.h"
#include "lifetime_box.h"
#include "memory.h"

namespace marlon {
namespace util {
template <typename T> class List {
public:
  using Const_iterator = T const *;
  using Iterator = T *;

  template <typename Allocator>
  static constexpr std::pair<Block, List> make(Allocator &allocator,
                                               Size max_size) {
    auto const block = allocator.alloc(memory_requirement(max_size));
    return {block, List{block, max_size}};
  }

  static constexpr Size memory_requirement(Size max_size) noexcept {
    return sizeof(T) * max_size;
  }

  constexpr List() noexcept
      : _begin{nullptr}, _stack_end{nullptr}, _buffer_end{nullptr} {}

  explicit List(Block block, Size max_size) noexcept
      : List{block.begin, max_size} {}

  explicit List(void *block_begin, Size max_size) noexcept
      : _begin{static_cast<T *>(block_begin)},
        _stack_end{_begin},
        _buffer_end{_begin + max_size} {}

  List(List<T> &&other)
      : _begin{std::exchange(other._begin, nullptr)},
        _stack_end{std::exchange(other._stack_end, nullptr)},
        _buffer_end{std::exchange(other._buffer_end, nullptr)} {}

  List &operator=(List<T> &&other) {
    auto temp = List<T>{std::move(other)};
    swap(temp);
    return *this;
  }

  ~List() { clear(); }

  T const &operator[](Size index) const noexcept { return _begin[index]; }

  T &operator[](Size index) noexcept { return _begin[index]; }

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

  Size size() const noexcept { return _stack_end - _begin; }

  Size max_size() const noexcept { return _buffer_end - _begin; }

  Size capacity() const noexcept { return max_size(); }

  void clear() noexcept {
    auto const begin = _begin;
    auto const end = _stack_end;
    for (auto it = begin; it != end; ++it) {
      it->~T();
    }
    _stack_end = _begin;
  }

  void push_back(T const &object) {
    if (_stack_end != _buffer_end) {
      new (_stack_end) T(object);
      ++_stack_end;
    } else {
      throw Capacity_error{"Capacity_error in List::push_back"};
    }
  }

  template <typename... Args> T &emplace_back(Args &&...args) {
    if (_stack_end != _buffer_end) {
      auto &result = *new (_stack_end) T(std::forward<Args>(args)...);
      ++_stack_end;
      return result;
    } else {
      throw Capacity_error{"Capacity_error in List::emplace_back"};
    }
  }

  void pop_back() noexcept { (--_stack_end)->~T(); }

  void resize(Size count) {
    auto const stack_end = _stack_end;
    auto const new_stack_end = _begin + count;
    if (new_stack_end > _buffer_end) {
      throw Capacity_error{"Capacity_error in List::resize"};
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
  void swap(List<T> &other) noexcept {
    std::swap(_begin, other._begin);
    std::swap(_stack_end, other._stack_end);
    std::swap(_buffer_end, other._buffer_end);
  }

  T *_begin;
  T *_stack_end;
  T *_buffer_end;
};

template <typename T, typename Allocator = System_allocator>
class Allocating_list : Allocator {
public:
  using Iterator = typename List<T>::Iterator;
  using Const_iterator = typename List<T>::Const_iterator;

  Allocating_list() { _impl.construct(); }

  explicit Allocating_list(Allocator const &allocator) : Allocator{allocator} {
    _impl.construct();
  }

  Allocating_list(Allocating_list &&other) noexcept
      : Allocator{std::move(static_cast<Allocator &>(other))} {
    _impl.construct(std::move(*other._impl));
  }

  Allocating_list &operator=(Allocating_list &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  ~Allocating_list() {
    if (_impl->data() != nullptr) {
      auto const block =
          make_block(_impl->data(), _impl->data() + _impl->max_size());
      _impl.destruct();
      Allocator::free(block);
    }
  }

  T const &operator[](Size index) const noexcept { return (*_impl)[index]; }

  T &operator[](Size index) noexcept { return (*_impl)[index]; }

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

  Size size() const noexcept { return _impl->size(); }

  Size max_size() const noexcept { return std::numeric_limits<Size>::max(); }

  Size capacity() const noexcept { return _impl->capacity(); }

  void reserve(Size capacity) {
    if (capacity > _impl->capacity()) {
      auto temp =
          List<T>::make(static_cast<Allocator &>(*this), capacity).second;
      for (auto &object : *_impl) {
        temp.emplace_back(std::move(object));
      }
      if (_impl->data() != nullptr) {
        auto const block =
            make_block(_impl->data(), _impl->data() + _impl->max_size());
        *_impl = std::move(temp);
        Allocator::free(block);
      } else {
        *_impl = std::move(temp);
      }
    }
  }

  void clear() { _impl->clear(); }

  void push_back(T const &object) {
    if (size() == capacity()) {
      reserve(size() != 0 ? size() * 2 : 1);
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

  void resize(Size count) {
    if (capacity() < count) {
      auto new_cap = std::max(capacity(), Size{1});
      while (new_cap < count) {
        new_cap *= 2;
      }
      reserve(new_cap);
    }
    _impl->resize(count);
  }

private:
  void swap(Allocating_list &other) noexcept {
    using std::swap;
    swap(static_cast<Allocator &>(*this), static_cast<Allocator &>(other));
    swap(*_impl, *other._impl);
  }

  Lifetime_box<List<T>> _impl;
};
} // namespace util
} // namespace marlon

#endif