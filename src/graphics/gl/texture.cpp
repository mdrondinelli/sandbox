#include "texture.h"

#include <cassert>

#include <stdexcept>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/glad.h>
#include <ktx.h>
#pragma clang diagnostic pop
#else
#include <glad/glad.h>
#include <ktx.h>
#endif

namespace marlon {
namespace graphics {
namespace {
ktxTexture *create_ktx_texture(Texture_create_info const &create_info) {
  ktxTexture2 *retval{};
  switch (create_info.source.index()) {
  case 0: {
    auto const &source = std::get<0>(create_info.source);
    assert(source.data != nullptr);
    assert(source.size != 0);
    auto const result = ktxTexture2_CreateFromMemory(
        static_cast<ktx_uint8_t const *>(source.data),
        static_cast<ktx_size_t>(source.size), KTX_TEXTURE_CREATE_NO_FLAGS,
        &retval);
    if (result != KTX_SUCCESS) {
      throw std::runtime_error{"Failed to create ktx texture"};
    }
    break;
  }
  case 1: {
    throw std::runtime_error{"Deprecated"};
  }
  default:
    throw std::runtime_error{"Unhandled switch case"};
  }
  if (ktxTexture2_NeedsTranscoding(retval)) {
    ktxTexture2_TranscodeBasis(retval, ktx_transcode_fmt_e::KTX_TTF_ETC2_RGBA,
                               0);
  }
  return reinterpret_cast<ktxTexture *>(retval);
}
} // namespace

Gl_texture::Gl_texture(Texture_create_info const &create_info) {
  auto const ktx_texture = create_ktx_texture(create_info);
  // TODO: handle errors here
  GLuint handle{};
  GLenum target{};
  auto const result =
      ktxTexture_GLUpload(ktx_texture, &handle, &target, nullptr);
  ktxTexture_Destroy(ktx_texture);
  if (result != KTX_SUCCESS) {
    throw std::runtime_error{"Failed to upload ktx texture."};
  }
  glTextureParameteri(handle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTextureParameterf(handle, GL_TEXTURE_MAX_ANISOTROPY, 16.0f);
  _handle = Gl_unique_texture_handle{handle};
}
} // namespace graphics
} // namespace marlon