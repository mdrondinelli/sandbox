#include "set.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("marlon::util::Set") {
  auto set = Set<void *>{};
  REQUIRE(set.begin() == set.cbegin());
  REQUIRE(set.end() == set.cend());
  REQUIRE(set.begin() == set.end());
  REQUIRE(set.begin() == set.cend());
  REQUIRE(set.cbegin() == set.end());
  REQUIRE(set.cbegin() == set.cend());
}
} // namespace util
} // namespace marlon