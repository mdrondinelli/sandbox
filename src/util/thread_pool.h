#ifndef MARLON_UTIL_THREAD_POOL_H
#define MARLON_UTIL_THREAD_POOL_H

#include <condition_variable>
#include <mutex>
#include <thread>

#include "list.h"
#include "queue.h"

namespace marlon {
namespace util {
class Task {
public:
  virtual ~Task() {}

  virtual void run(unsigned thread_index) = 0;
};

class Thread_pool {
public:
  explicit Thread_pool(unsigned thread_count);

  ~Thread_pool();

  std::size_t size() const noexcept;

  void push(Task *task);

private:
  class Thread {
  public:
    explicit Thread(Thread *threads, unsigned thread_count, unsigned index);

    ~Thread();

    void push(Task *task);

    bool try_push(Task *task);

  private:
    Thread *_threads;
    unsigned _thread_count;
    unsigned _index;
    Allocating_queue<Task *> _queue;
    std::mutex _mutex;
    std::condition_variable _condvar;
    std::jthread _thread;
  };

  List<Thread> _threads;
  std::atomic<std::size_t> _push_index;
};
} // namespace util
} // namespace marlon

#endif