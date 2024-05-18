#include "uniform_buffer.h"

#include <glad/gl.h>

namespace marlon::graphics::gl {
Uniform_buffer::Uniform_buffer(Uniform_buffer_create_info const &create_info)
    : _buffer{wrappers::make_unique_buffer()} {
  glNamedBufferStorage(_buffer.get(),
                       create_info.size,
                       nullptr,
                       GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT |
                           GL_MAP_COHERENT_BIT);
  _data =
      static_cast<std::byte *>(glMapNamedBuffer(_buffer.get(), GL_WRITE_ONLY));
}
} // namespace marlon::graphics::gl