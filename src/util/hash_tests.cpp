#include "hash.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("marlon::util::Hash") {
  int a, b;
  REQUIRE(Hash<std::nullptr_t>{}(nullptr) == 0);
  REQUIRE(Hash<int *>{}(&a) != Hash<int *>{}(&b));
}
} // namespace util
} // namespace marlon