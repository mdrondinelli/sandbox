#ifndef MARLON_UTIL_THREAD_POOL_H
#define MARLON_UTIL_THREAD_POOL_H

#include <thread>

#include "array.h"

namespace marlon {
namespace util {
class Thread_pool {
public:
  enum class Synchronization_policy { spin, block };

  class Task {
  public:
    virtual ~Task() {}

    virtual void run() = 0;
  };

  static constexpr std::size_t memory_requirement(std::size_t thread_count,
                                                  std::size_t max_queue_size) {}

  explicit Thread_pool(Block block, std::size_t thread_count,
                       std::size_t max_queue_size,
                       Synchronization_policy synchronization_policy)
      : Thread_pool{block.begin, thread_count, max_queue_size,
                     synchronization_policy} {}

  explicit Thread_pool(void const *block_begin, std::size_t thread_count,
                       std::size_t max_queue_size,
                       Synchronization_policy synchronization_policy);

private:
  class Node {
  public:
    Node();

  private:
    std::thread _thread;
  };

  List<Node> _nodes;
};
} // namespace util
} // namespace marlon

#endif