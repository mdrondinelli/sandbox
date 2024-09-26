#include "timeout.h"

#include <array>
#include <iostream>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace marlon::game {
TEST_CASE("Timemouts work") {
  struct Callback : Timeout_callback {
    std::string *s;
    char c;

    Callback(std::string *s, char c) noexcept
        : s{s}, c{c} {}

    void on_timeout() {
      *s += c;
    }
  };

  auto timeouts =
      Timeout_manager{{.max_timeouts = 1000, .max_intervals = 1000}};
  auto test_string = std::string{};
  auto test_callbacks = std::array<Callback, 5>{
      Callback{&test_string, 'h'},
      Callback{&test_string, 'e'},
      Callback{&test_string, 'l'},
      Callback{&test_string, 'l'},
      Callback{&test_string, 'o'},
  };
  for (auto i = 0; i < 5; ++i) {
    timeouts.set_timeout(&test_callbacks[i], i * 0.1);
  }
  for (auto i = 0; i < 3; ++i) {
    timeouts.on_time_passing(1.0 / 3.0);
  }
  REQUIRE(test_string == "hello");
  test_string = "";
  timeouts.set_interval(&test_callbacks[0], 0.1);
  timeouts.on_time_passing(0.01);
  timeouts.set_interval(&test_callbacks[1], 0.1);
  timeouts.on_time_passing(1.01);
  REQUIRE(test_string == "hehehehehehehehehehe");
}
} // namespace marlon::game
