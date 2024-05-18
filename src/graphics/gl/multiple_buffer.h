#ifndef MARLON_GRAPHICS_GL_MULTIPLE_BUFFER_H
#define MARLON_GRAPHICS_GL_MULTIPLE_BUFFER_H

#include <array>

#include "wrappers/wrappers.h"

namespace marlon::graphics::gl {
template <typename T, int N> class Multiple_buffer {
public:
  Multiple_buffer() = default;

  template <typename... Args> explicit Multiple_buffer(Args &&...args) {
    for (auto &resource : _resources) {
      resource.resource = T(std::forward<Args>(args)...);
    }
  }

  void acquire() {
    _index = (_index + 1) % N;
    if (_resources[_index].sync) {
      _resources[_index].sync.wait(true);
      _resources[_index].sync = {};
    }
  }

  void release() { _resources[_index].sync = wrappers::make_unique_sync(); }

  T const &get() const noexcept { return _resources[_index].resource; }

  T &get() noexcept { return _resources[_index].resource; }

private:
  struct Synchronized_resource {
    T resource;
    wrappers::Unique_sync sync;
  };

  std::array<Synchronized_resource, N> _resources;
  int _index{N - 1};
};

template <typename T> using Double_buffer = Multiple_buffer<T, 2>;

template <typename T> using Triple_buffer = Multiple_buffer<T, 3>;
} // namespace marlon::graphics::gl

#endif