#include "array.h"

#include <array>

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
// TEST_CASE("Stacks can be constructed from a block and a capacity.") {
//   alignas(16) std::array<std::byte, 1024> memory;
//   make_block(memory.data(), memory.size());
// }
TEST_CASE("Dynamic_array") {
  auto a = Dynamic_array<int>{System_allocator::instance()};
  for (int i = 0; i < 100000; ++i) {
    a.emplace_back(i);
  }
  for (int i = 0; i < 100000; ++i) {
    REQUIRE(a[i] == i);
  }
}
} // namespace util
} // namespace marlon