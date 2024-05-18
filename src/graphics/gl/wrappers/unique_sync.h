#ifndef MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_SYNC_H
#define MARLON_GRAPHICS_GL_WRAPPERS_UNIQUE_SYNC_H

#include <cstdint>

#include <utility>

namespace marlon::graphics::gl::wrappers {
class Unique_sync {
public:
  constexpr Unique_sync() noexcept = default;

  explicit Unique_sync(void *sync) noexcept : _sync{sync} {}

  ~Unique_sync();

  constexpr Unique_sync(Unique_sync &&other) noexcept
      : _sync{std::exchange(other._sync, nullptr)} {}

  Unique_sync &operator=(Unique_sync &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  constexpr operator bool() const noexcept { return _sync != nullptr; }

  constexpr void *get() const noexcept { return _sync; }

  void wait(bool flush) const noexcept;

private:
  void swap(Unique_sync &other) noexcept { std::swap(_sync, other._sync); }

  void *_sync{};
};

Unique_sync make_unique_sync();
} // namespace marlon::graphics::gl::wrappers

#endif