#include "thread_pool.h"

#include <iostream>
#include <random>
#include <stop_token>

namespace marlon {
namespace util {
Thread_pool::Thread_pool(Size thread_count, Scheduling_policy scheduling_policy)
    : _push_index{Size{}} {
  if (thread_count > 0) {
    auto block = Block{};
    std::tie(block, _threads) =
        List<Thread>::make(*System_allocator::instance(), thread_count);
    try {
      for (int i = 0; i != thread_count; ++i) {
        _threads.emplace_back(
            _threads.data(), thread_count, i, scheduling_policy);
      }
    } catch (...) {
      _threads = {};
      System_allocator::instance()->free(block);
      throw;
    }
  }
}

Thread_pool::~Thread_pool() {
  if (!empty()) {
    auto const block = make_block(
        _threads.data(), List<Thread>::memory_requirement(_threads.max_size()));
    _threads = {};
    System_allocator::instance()->free(block);
  }
}

bool Thread_pool::empty() const noexcept { return size() <= 0; }

Size Thread_pool::size() const noexcept {
  return _threads.size();
}

Size Thread_pool::push_notify(Task *task) {
  auto const index = push_silent(task);
  _threads[index].notify();
  return index;
}

Size Thread_pool::push_silent(Task *task) {
  auto const base = _push_index++;
  for (auto offset = Size{}; offset != 2 * _threads.size(); ++offset) {
    auto const index = (base + offset) % _threads.size();
    auto &thread = _threads[index];
    if (thread.try_push(task)) {
      return index;
    }
  }
  auto const index = base % _threads.size();
  auto &thread = _threads[index];
  thread.push(task);
  return index;
}

void Thread_pool::notify() {
  for (auto &thread : _threads) {
    thread.notify();
  }
}

void Thread_pool::set_scheduling_policy(Scheduling_policy scheduling_policy) {
  for (auto &thread : _threads) {
    thread.set_scheduling_policy(scheduling_policy);
  }
}

Thread_pool::Thread::Thread(Thread *threads,
                            Size thread_count,
                            Size index,
                            Scheduling_policy scheduling_policy)
    : _threads{threads},
      _thread_count{thread_count},
      _index{index},
      _scheduling_policy{scheduling_policy},
      _queue{System_allocator::instance()},
      _thread{[this](std::stop_token stop_token) {
        {
          auto const lock = std::scoped_lock{_mutex};
          _queue.reserve(1024);
        }
        auto rng = std::minstd_rand{std::random_device{}()};
        auto try_steal = [&](int attempts) -> Task * {
          auto d = std::uniform_int_distribution<Size>{
              Size{1}, std::max(Size{1}, _thread_count - 1)};
          for (auto attempt = 0; attempt < attempts; ++attempt) {
            auto const offset = d(rng);
            auto const index = (_index + offset) % _thread_count;
            auto &thread = _threads[index];
            if (auto lock = std::unique_lock{thread._mutex, std::try_to_lock}) {
              if (!thread._queue.empty()) {
                auto result = thread._queue.front();
                thread._queue.pop_front();
                return result;
              }
            }
          }
          return nullptr;
        };
        for (;;) {
          // try to steal from our own queue
          auto task = (Task *)nullptr;
          if (auto lock = std::unique_lock{_mutex, std::try_to_lock}) {
            if (!_queue.empty()) {
              task = _queue.back();
              _queue.pop_back();
            }
          }
          // try to steal from other queues
          if (!task) {
            task = try_steal(_thread_count - 1);
          }
          // just wait for work
          if (!task) {
            auto lock = std::unique_lock{_mutex};
            _condvar.wait(lock, [&] {
              return _scheduling_policy == Scheduling_policy::spin ||
                     !_queue.empty() || stop_token.stop_requested();
            });
            if (!_queue.empty()) {
              task = _queue.back();
              _queue.pop_back();
            }
          }
          if (task) {
            try {
              task->run(_index);
            } catch (std::exception &e) {
              std::cerr << "Exception in Task::run: " << e.what() << "\n";
            } catch (...) {
              std::cerr << "Exception in Task::run\n";
            }
          } else if (stop_token.stop_requested()) {
            return;
          }
        }
      }} {}

Thread_pool::Thread::~Thread() {
  _thread.request_stop();
  _condvar.notify_one();
}

void Thread_pool::Thread::push(Task *task) {
  auto lock = std::unique_lock{_mutex};
  _queue.push_back(task);
}

bool Thread_pool::Thread::try_push(Task *task) {
  if (auto lock = std::unique_lock{_mutex, std::try_to_lock}) {
    _queue.push_back(task);
    return true;
  } else {
    return false;
  }
}

void Thread_pool::Thread::notify() {
  _condvar.notify_one();
}

void Thread_pool::Thread::set_scheduling_policy(
    Scheduling_policy scheduling_policy) {
  {
    auto const lock = std::scoped_lock{_mutex};
    _scheduling_policy = scheduling_policy;
  }
  notify();
}
} // namespace util
} // namespace marlon