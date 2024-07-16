#include "queue.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("marlon::util::Queue") {
  auto q = Allocating_queue<int>{};
  REQUIRE(q.size() == 0);
  REQUIRE(q.empty());
  REQUIRE(q.begin() == q.end());
  for (int i = 0; i < 100000; ++i) {
    q.emplace_back(i);
  }
  REQUIRE(q.size() == 100000);
  for (int i = 0; i < 100000; ++i) {
    auto n = q.front();
    REQUIRE(n == i);
    q.pop_front();
  }
  REQUIRE(q.size() == 0);
  REQUIRE(q.empty());
  REQUIRE(q.begin() == q.end());
  for (int i = 0; i < 100000; ++i) {
    q.emplace_front(i);
  }
  for (int i = 0; i < 100000; ++i) {
    auto n = q.back();
    REQUIRE(n == i);
    q.pop_back();
  }
}
} // namespace util
} // namespace marlon