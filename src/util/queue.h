#ifndef MARLON_UTIL_QUEUE_H
#define MARLON_UTIL_QUEUE_H

#include <cstdint>

namespace marlon {
namespace util {
template <typename T>
class Queue {
public:

private:
  T *_data;
  std::size_t _max_size;
  std::size_t _write_index;
  std::size_t _read_index;
};
} // namespace util
} // namespace marlon

#endif