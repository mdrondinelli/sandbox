#include <catch2/catch_test_macros.hpp>

#include "bit_list.h"

namespace marlon {
namespace util {
TEST_CASE("marlon::util::Bit_list") {
  auto constexpr requested_max_size = std::size_t{100};
  auto [block, bit_list] =
      Bit_list::make(*System_allocator::instance(), requested_max_size);
  REQUIRE(bit_list.max_size() >= requested_max_size);
  REQUIRE(bit_list.capacity() >= requested_max_size);
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    bit_list.push_back(true);
  }
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    REQUIRE(bit_list.get(i) == true);
  }
  REQUIRE(bit_list.size() == requested_max_size);
  bit_list.clear();
  REQUIRE(bit_list.size() == 0);
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    bit_list.push_back(false);
  }
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    REQUIRE(bit_list.get(i) == false);
  }
  REQUIRE(bit_list.size() == requested_max_size);
  bit_list.clear();
  REQUIRE(bit_list.size() == 0);
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    bit_list.push_back(i % 2 == 0);
  }
  REQUIRE(bit_list.size() == requested_max_size);
  bit_list.resize(16);
  REQUIRE(bit_list.size() == 16);
  bit_list.resize(requested_max_size);
  REQUIRE(bit_list.size() == requested_max_size);
  for (auto i{std::size_t{}}; i != requested_max_size; ++i) {
    if (i < 16) {
      REQUIRE(bit_list.get(i) == (i % 2 == 0));
    } else {
      REQUIRE(bit_list.get(i) == false);
    }
  }
  System_allocator::instance()->free(block);
}
} // namespace util
} // namespace marlon