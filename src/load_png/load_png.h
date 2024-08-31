#ifndef MARLON_LOAD_PNG_H
#define MARLON_LOAD_PNG_H

namespace marlon::load_png {
enum class Load_png_result {
  success,
  file_open_failed,
  png_structure_allocation_failed,

};
Load_png_result load_png(char const *path);
} // namespace marlon::load_png

#endif
