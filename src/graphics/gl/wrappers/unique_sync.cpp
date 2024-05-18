#include "unique_sync.h"

#include <glad/gl.h>

namespace marlon::graphics::gl::wrappers {
Unique_sync::~Unique_sync() { glDeleteSync(static_cast<GLsync>(_sync)); }

void Unique_sync::wait(bool flush) const noexcept {
  for (;;) {
    GLenum waitReturn = glClientWaitSync(
        static_cast<GLsync>(_sync), flush ? GL_SYNC_FLUSH_COMMANDS_BIT : 0, 1);
    if (waitReturn == GL_ALREADY_SIGNALED ||
        waitReturn == GL_CONDITION_SATISFIED) {
      return;
    }
  }
}

Unique_sync make_unique_sync() {
  return Unique_sync{glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)};
}
} // namespace marlon::graphics::gl::wrappers