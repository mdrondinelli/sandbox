#ifndef MARLON_UTIL_THREAD_POOL_H
#define MARLON_UTIL_THREAD_POOL_H

#include <condition_variable>
#include <mutex>
#include <thread>

#include "list.h"
#include "queue.h"

namespace marlon {
namespace util {
enum class Scheduling_policy { block, spin };

class Task {
public:
  virtual ~Task() {}

  virtual void run(unsigned thread_index) = 0;
};

class Thread_pool {
public:
  explicit Thread_pool(
      unsigned thread_count,
      Scheduling_policy scheduling_policy = Scheduling_policy::block);

  ~Thread_pool();

  std::size_t size() const noexcept;

  void push(Task *task);

  // CANNOT be called from inside the thread pool
  void set_scheduling_policy(Scheduling_policy policy);

private:
  class Thread {
  public:
    explicit Thread(Thread *threads,
                    unsigned thread_count,
                    unsigned index,
                    Scheduling_policy scheduling_policy);

    ~Thread();

    void push(Task *task);

    bool try_push(Task *task);

    void set_scheduling_policy(Scheduling_policy scheduling_policy);

  private:
    Thread *const _threads;
    unsigned const _thread_count;
    unsigned const _index;
    Scheduling_policy _scheduling_policy;
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