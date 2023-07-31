#include "texture.h"

#include <stdexcept>

#include <ktx.h>

namespace marlon {
namespace graphics {
namespace {
ktxTexture *create_ktx_texture(Texture_create_info const &create_info) {
  ktxTexture *retval;
  switch (create_info.source.index()) {
  case 0: {
    auto const &source = std::get<0>(create_info.source);
    ktxTexture_CreateFromMemory(static_cast<ktx_uint8_t const *>(source.data),
                                static_cast<ktx_size_t>(source.size),
                                KTX_TEXTURE_CREATE_NO_FLAGS, &retval);
    break;
  }
  case 1: {
    auto const &source = std::get<1>(create_info.source);
    ktxTexture_CreateFromStdioStream(source.file, KTX_TEXTURE_CREATE_NO_FLAGS,
                                     &retval);
    break;
  }
  default:
    throw std::runtime_error{"Unhandled switch case"};
  }
  return retval;
}
} // namespace

Gl_texture::Gl_texture(Texture_create_info const &create_info) {
  auto const ktx_texture = create_ktx_texture(create_info);
  // TODO: handle errors here
  GLuint handle;
  GLenum target;
  ktxTexture_GLUpload(ktx_texture, &handle, &target, nullptr);
  _handle = Gl_unique_texture_handle{handle};
  ktxTexture_Destroy(ktx_texture);
}
} // namespace graphics
} // namespace marlon