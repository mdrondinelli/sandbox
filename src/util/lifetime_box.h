#ifndef MARLON_UTIL_LIFETIME_BOX_H
#define MARLON_UTIL_LIFETIME_BOX_H

#include <array>
#include <new>

namespace marlon {
namespace util {
template <typename T> class Lifetime_box {
public:
  Lifetime_box() = default;

  Lifetime_box(Lifetime_box<T> const &other) = delete;

  Lifetime_box &operator=(Lifetime_box<T> const &other) = delete;

  template <typename... Args> T &construct(Args &&...args) {
    return *new (&_storage) T(std::forward<Args>(args)...);
  }

  void destruct() noexcept {
    get()->~T();
  }

  T const *get() const noexcept {
    return std::launder(reinterpret_cast<T const *>(&_storage));
  }

  T *get() noexcept { return std::launder(reinterpret_cast<T *>(&_storage)); }

  T const &operator*() const noexcept {
    return *get();
  }

  T &operator*() noexcept {
    return *get();
  }

  T const *operator->() const noexcept {
    return get();
  }

  T *operator->() noexcept {
    return get();
  }

private:
  alignas(alignof(T)) std::array<std::byte, sizeof(T)> _storage;
};
} // namespace util
} // namespace marlon

#endif