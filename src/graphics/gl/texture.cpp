#include "texture.h"

#include <cassert>

#include <stdexcept>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wlanguage-extension-token"
#include <glad/gl.h>
#include <ktx.h>
#pragma clang diagnostic pop
#else
#include <glad/gl.h>
#include <ktx.h>
#endif

namespace marlon {
namespace graphics {
namespace gl {
namespace {
ktxTexture2 *create_ktx_texture(Texture_create_info const &create_info) {
  assert(create_info.data != nullptr);
  assert(create_info.size != 0);
  ktxTexture2 *retval{};
  auto const result = ktxTexture2_CreateFromMemory(
      static_cast<ktx_uint8_t const *>(create_info.data),
      static_cast<ktx_size_t>(create_info.size),
      KTX_TEXTURE_CREATE_NO_FLAGS,
      &retval);
  if (result != KTX_SUCCESS) {
    throw std::runtime_error{"Failed to create ktx texture"};
  }
  if (ktxTexture2_NeedsTranscoding(retval)) {
    ktxTexture2_TranscodeBasis(
        retval, ktx_transcode_fmt_e::KTX_TTF_ETC2_RGBA, 0);
  }
  return retval;
}
} // namespace

Texture::Texture(Texture_create_info const &create_info) {
  auto const ktx_texture = create_ktx_texture(create_info);
  // TODO: handle errors here
  GLuint handle{};
  GLenum target{};
  auto const result = ktxTexture_GLUpload(
      reinterpret_cast<ktxTexture *>(ktx_texture), &handle, &target, nullptr);
  ktxTexture_Destroy(reinterpret_cast<ktxTexture *>(ktx_texture));
  if (result != KTX_SUCCESS) {
    throw std::runtime_error{"Failed to upload ktx texture."};
  }
  glTextureParameteri(handle, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTextureParameterf(handle, GL_TEXTURE_MAX_ANISOTROPY, 16.0f);
  _handle = wrappers::Unique_texture{handle};
}
} // namespace gl
} // namespace graphics
} // namespace marlon