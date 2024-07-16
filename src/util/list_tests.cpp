#include "list.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("Allocating_list") {
  auto a = Allocating_list<int>{};
  for (int i = 0; i < 100000; ++i) {
    a.emplace_back(i);
  }
  for (int i = 0; i < 100000; ++i) {
    REQUIRE(a[i] == i);
  }
}
} // namespace util
} // namespace marlon