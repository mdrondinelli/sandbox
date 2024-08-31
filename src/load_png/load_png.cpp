#include "load_png.h"

#include <cstdio>

#include <new>
#include <utility>

#include <png.h>

namespace marlon::load_png {
namespace {
class Unique_file {
public:
  constexpr Unique_file() noexcept = default;

  explicit Unique_file(char const *filename, char const *mode)
      : _file{std::fopen(filename, mode)} {}

  ~Unique_file() {
    if (_file) {
      std::fclose(_file);
    }
  }

  constexpr Unique_file(Unique_file &&other) noexcept
      : _file{std::exchange(other._file, nullptr)} {}

  Unique_file &operator=(Unique_file &&other) noexcept {
    auto temp{std::move(other)};
    swap(temp);
    return *this;
  }

  constexpr operator bool() const noexcept {
    return _file != nullptr;
  }

  constexpr std::FILE *get() const noexcept {
    return _file;
  }

private:
  constexpr void swap(Unique_file &other) noexcept {
    std::swap(_file, other._file);
  }

  std::FILE *_file{nullptr};
};

class Unique_png_struct {
public:
  constexpr Unique_png_struct() = default;

  explicit Unique_png_struct(char const *libpng_version_string)
      : _png_ptr{png_create_read_struct(libpng_version_string, nullptr, nullptr, nullptr)} {
    if (!_png_ptr) {
      throw std::bad_alloc{};
    }
    _info_ptr = png_create_info_struct(_png_ptr);
    if (!_info_ptr) {
      png_destroy_read_struct(&_png_ptr, nullptr, nullptr);
    }
    _end_info = png_create_info_struct(_png_ptr);
    if (!_end_info) {
      png_destroy_read_struct(&_png_ptr, &_info_ptr, nullptr);
    }
  }

  ~Unique_png_struct() {
    if (_png_ptr) {
      png_destroy_read_struct(&_png_ptr, &_info_ptr, &_end_info);
    }
  }

private:
  png_structp _png_ptr{nullptr};
  png_infop _info_ptr{nullptr};
  png_infop _end_info{nullptr};
};
} // namespace

Load_png_result load_png(char const *path) {
  auto const file = Unique_file{path, "rb"};
  if (!file) {
    return Load_png_result::file_open_failed;
  }
  auto const png_ptr = Unique_png_struct{PNG_LIBPNG_VER_STRING};
  return Load_png_result::success;
}
} // namespace marlon::load_png
