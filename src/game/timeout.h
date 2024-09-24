#ifndef MARLON_GAME_TIMEOUT_CALLBACK_H
#define MARLON_GAME_TIMEOUT_CALLBACK_H

#include <util/list.h>
#include <util/pool.h>
#include <util/set.h>

namespace marlon::game {
class Timeout_callback {
public:
  virtual ~Timeout_callback() = default;

  virtual void on_timeout() = 0;
};

struct Timeout {
  Timeout_callback *callback;
  double delay;
};

struct Interval {
  Timeout_callback *callback;
  double delay;
  double initial_delay;
};

struct Timeout_manager_create_info {
  util::Size max_timeouts;
  util::Size max_intervals;
};

class Timeout_manager {
public:
  Timeout_manager() = default;

  explicit Timeout_manager(Timeout_manager_create_info const &create_info);

  ~Timeout_manager();

  Timeout *set_timeout(Timeout_callback *callback, double delay);

  Interval *set_interval(Timeout_callback *callback, double delay);

  void clear(Timeout *timeout);

  void clear(Interval *interval);

  void on_time_passing(double dt);

private:
  util::Block _memory;
  util::Pool<Timeout> _timeouts;
  util::Pool<Interval> _intervals;
  util::Set<Timeout *> _allocated_timeouts;
  util::Set<Interval *> _allocated_intervals;
  util::List<Timeout *> _expiring_timeouts;
  util::List<Interval *> _expiring_intervals;
};
} // namespace marlon::game

#endif
