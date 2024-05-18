#ifndef MARLON_GRAPHICS_GL_UNIFORM_BUFFER_H
#define MARLON_GRAPHICS_GL_UNIFORM_BUFFER_H

#include <cstddef>

#include "wrappers/wrappers.h"

namespace marlon::graphics::gl {
struct Uniform_buffer_create_info {
  int size;
};

class Uniform_buffer {
public:
  constexpr Uniform_buffer() = default;

  explicit Uniform_buffer(Uniform_buffer_create_info const &create_info);

  constexpr std::uint32_t get() const noexcept { return _buffer.get(); }
  
  constexpr std::byte *data() const noexcept { return _data; }

private:
  wrappers::Unique_buffer _buffer;
  std::byte *_data{};
};
} // namespace marlon::graphics::gl

#endif