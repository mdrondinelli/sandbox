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

  virtual void run(Size thread_index) = 0;
};

class Thread_pool {
public:
  explicit Thread_pool(
      Size thread_count,
      Scheduling_policy scheduling_policy = Scheduling_policy::block);

  ~Thread_pool();

  bool empty() const noexcept;

  Size size() const noexcept;

  Size push_notify(Task *task);

  Size push_silent(Task *task);

  void notify();

  // CANNOT be called from inside the thread pool
  void set_scheduling_policy(Scheduling_policy policy);

private:
  class Thread {
  public:
    explicit Thread(Thread *threads,
                    Size thread_count,
                    Size index,
                    Scheduling_policy scheduling_policy);

    ~Thread();

    void push(Task *task);

    bool try_push(Task *task);

    void notify();

    void set_scheduling_policy(Scheduling_policy scheduling_policy);

  private:
    Thread *const _threads;
    Size const _thread_count;
    Size const _index;
    Scheduling_policy _scheduling_policy;
    Allocating_queue<Task *> _queue;
    std::mutex _mutex;
    std::condition_variable _condvar;
    std::jthread _thread;
  };

  List<Thread> _threads;
  std::atomic<Size> _push_index;
};
} // namespace util
} // namespace marlon

#endif