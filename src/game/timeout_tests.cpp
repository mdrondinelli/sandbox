#include "timeout.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon::game {
TEST_CASE("Timemouts work") {
  auto timeouts =
      Timeout_manager{{.max_timeouts = 1000, .max_intervals = 1000}};
}
} // namespace marlon::game
