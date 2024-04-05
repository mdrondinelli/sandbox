#ifndef MARLON_GRAPHICS_TEXTURE_H
#define MARLON_GRAPHICS_TEXTURE_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <variant>

namespace marlon {
namespace graphics {
struct Texture_memory_source {
  void const *data;
  std::size_t size;
};

struct Texture_stream_source {
  std::FILE *file;
};

struct Texture_create_info {
  std::variant<Texture_memory_source, Texture_stream_source> source;
};

class Texture {};
} // namespace graphics
} // namespace marlon

#endif