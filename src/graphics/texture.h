#ifndef MARLON_GRAPHICS_TEXTURE_H
#define MARLON_GRAPHICS_TEXTURE_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <variant>

namespace marlon {
namespace graphics {
struct Texture_create_info {
  void const *data;
  std::size_t size;
};

class Texture {};
} // namespace graphics
} // namespace marlon

#endif