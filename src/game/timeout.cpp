#include <algorithm>
#include <optional>
#include <variant>

#include "timeout.h"
#include "util/memory.h"

namespace marlon::game {
Timeout_manager::Timeout_manager(
    Timeout_manager_create_info const &create_info) {
  _memory = util::assign_merged(util::System_allocator{},
                                std::tie(_timeouts,
                                         _allocated_timeouts,
                                         _expiring_timeouts,
                                         _intervals,
                                         _allocated_intervals,
                                         _expiring_intervals),
                                std::tuple{create_info.max_timeouts},
                                std::tuple{create_info.max_timeouts},
                                std::tuple{create_info.max_timeouts},
                                std::tuple{create_info.max_intervals},
                                std::tuple{create_info.max_intervals},
                                std::tuple{create_info.max_intervals});
}

Timeout_manager::~Timeout_manager() {
  util::System_allocator{}.free(_memory);
}

Timeout *Timeout_manager::set_timeout(Timeout_callback *callback,
                                      double delay) {
  auto const result =
      _timeouts.emplace(Timeout{.callback = callback, .delay = delay});
  _allocated_timeouts.emplace(result);
  return result;
}

Interval *Timeout_manager::set_interval(Timeout_callback *callback,
                                        double delay) {
  auto const result = _intervals.emplace(Interval{
      .callback = callback,
      .delay = delay,
      .initial_delay = delay,
  });
  _allocated_intervals.emplace(result);
  return result;
}

void Timeout_manager::clear(Timeout *timeout) {
  _timeouts.erase(timeout);
  _allocated_timeouts.erase(timeout);
}

void Timeout_manager::clear(Interval *interval) {
  _intervals.erase(interval);
  _allocated_intervals.erase(interval);
}

void Timeout_manager::on_time_passing(double dt) {
  for (auto it = _allocated_timeouts.begin(); it != _allocated_timeouts.end();
       ++it) {
    if (((*it)->delay -= dt) < 0) {
      _expiring_timeouts.emplace_back(it);
    }
  }
  for (auto it = _allocated_intervals.begin(); it != _allocated_intervals.end();
       ++it) {
    if (((*it)->delay -= dt) < 0) {
      _expiring_intervals.emplace_back(it);
    }
  }
  auto const comparator = [](auto const &lhs, auto const &rhs) {
    return lhs->delay < rhs->delay;
  };
  std::ranges::sort(_expiring_timeouts, comparator);
  std::ranges::sort(_expiring_intervals, comparator);
  auto expiring_timeout_it = _expiring_timeouts.begin();
  auto expiring_interval_it = _expiring_intervals.begin();
  for (;;) {
    using Choice = std::variant<Timeout **, Interval **>;
    auto choice = std::optional<Choice>{};
    auto const choice_delay = [&]() {
      return std::visit([](auto const choice) { return (*choice)->delay; },
                        *choice);
    };
    if (expiring_timeout_it != _expiring_timeouts.end()) {
      choice = expiring_timeout_it;
    }
    if (expiring_interval_it != _expiring_intervals.end()) {
      if (!choice || (*expiring_interval_it)->delay < choice_delay()) {
        choice = expiring_interval_it;
      }
    }
    if (choice) {
      std::visit(
          [&](auto const choice) {
            (*choice)->callback->on_timeout();
            using T = std::decay_t<decltype(choice)>;
            if constexpr (std::is_same_v<T, Timeout **>) {
              ++expiring_timeout_it;
            } else {
              if (((*choice)->delay += (*choice)->initial_delay) < 0) {
                // chosen (most negative) interval timeout needs to be fired
                // again, so we bubble it toward the end
                auto first = choice;
                auto const last = _expiring_intervals.end();
                for (;;) {
                  auto const next = first + 1;
                  if (next < last) {
                    if ((*next)->delay < (*first)->delay) {
                      std::swap(*first, *next);
                      ++first;
                    } else {
                      break;
                    }
                  } else {
                    break;
                  }
                }
              }
            }
          },
          *choice);
    } else {
      break;
    }
  }
  _expiring_timeouts.clear();
  _expiring_intervals.clear();
}
} // namespace marlon::game
