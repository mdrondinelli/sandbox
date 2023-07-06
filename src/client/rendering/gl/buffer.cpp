#include "buffer.h"

#include <glad/glad.h>

namespace marlon {
namespace rendering {
Gl_buffer::Gl_buffer(Gl_default_handle_init) { glCreateBuffers(1, &_handle); }

Gl_buffer::~Gl_buffer() { glDeleteBuffers(1, &_handle); }
} // namespace rendering
} // namespace marlon