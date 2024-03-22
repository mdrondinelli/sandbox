#include "thread_pool.h"

namespace marlon {
namespace util {
Thread_pool::Thread_pool(void const *block_begin, std::size_t thread_count,
                         std::size_t max_queue_size,
                         Synchronization_policy synchronization_policy) {}
} // namespace util
} // namespace marlon