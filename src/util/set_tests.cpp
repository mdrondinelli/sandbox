#include "set.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
// TEST_CASE("marlon::util::Set") {
//   auto const max_bucket_count = 64;
//   auto const max_node_count = 64;
//   auto const block = make_unique_block(
//       Set<int>::memory_requirement(max_node_count, max_bucket_count));
//   auto set = Set<int>{};
//   REQUIRE(set.begin() == set.cbegin());
//   REQUIRE(set.end() == set.cend());
//   REQUIRE(set.begin() == set.end());
//   REQUIRE(set.begin() == set.cend());
//   REQUIRE(set.cbegin() == set.end());
//   REQUIRE(set.cbegin() == set.cend());
//   REQUIRE(set.size() == 0);
//   set = Set<int>{block.get(), max_node_count, max_bucket_count};
//   for (auto j = 0; j < 2; ++j) {
//     for (int i = 1; i <= 63; ++i) {
//       auto const it = set.emplace(i);
//       REQUIRE(*it.first == i);
//       REQUIRE(it.second);
//       REQUIRE(set.size() == static_cast<std::size_t>(i));
//     }
//     for (int i = 1; i <= 63; ++i) {
//       auto const it = set.emplace(i);
//       REQUIRE(*it.first == i);
//       REQUIRE(!it.second);
//       REQUIRE(set.size() == 63);
//     }
//     set.emplace(64);
//     REQUIRE(set.size() == 64);
//     REQUIRE_NOTHROW([&]() { set.emplace(64); }());
//     set.insert(64);
//     REQUIRE(set.size() == 64);
//     set.clear();
//     REQUIRE(set.size() == 0);
//   }
//   for (int i = 1; i <= 64; ++i) {
//     auto const it = set.insert(i);
//     REQUIRE(*it.first == i);
//     REQUIRE(it.second);
//     REQUIRE(set.size() == static_cast<std::size_t>(i));
//   }
//   for (int i = 1; i <= 64; ++i) {
//     auto const it = set.insert(i);
//     REQUIRE(*it.first == i);
//     REQUIRE(!it.second);
//     REQUIRE(set.size() == 64);
//   }
//   std::vector<bool> v(64);
//   for (auto const i : set) {
//     v[i - 1] = true;
//   }
//   for (auto const b : v) {
//     REQUIRE(b);
//   }
// }
// TEST_CASE("marlon::util::Allocating_set") {
//   auto set = Allocating_set<int>{System_allocator::instance()};
//   REQUIRE(set.begin() == set.cbegin());
//   REQUIRE(set.end() == set.cend());
//   REQUIRE(set.begin() == set.end());
//   REQUIRE(set.begin() == set.cend());
//   REQUIRE(set.cbegin() == set.end());
//   REQUIRE(set.cbegin() == set.cend());
//   REQUIRE(set.size() == 0);
//   for (auto j = 0; j < 2; ++j) {
//     for (int i = 1; i <= 63; ++i) {
//       auto const it = set.emplace(i);
//       REQUIRE(*it.first == i);
//       REQUIRE(it.second);
//       REQUIRE(set.size() == static_cast<std::size_t>(i));
//     }
//     for (int i = 1; i <= 63; ++i) {
//       auto const it = set.emplace(i);
//       REQUIRE(*it.first == i);
//       REQUIRE(!it.second);
//       REQUIRE(set.size() == 63);
//     }
//     set.emplace(64);
//     REQUIRE(set.size() == 64);
//     set.insert(64);
//     REQUIRE(set.size() == 64);
//     set.clear();
//     REQUIRE(set.size() == 0);
//   }
//   for (int i = 1; i <= 64; ++i) {
//     auto const it = set.insert(i);
//     REQUIRE(*it.first == i);
//     REQUIRE(it.second);
//     REQUIRE(set.size() == static_cast<std::size_t>(i));
//   }
//   for (int i = 1; i <= 64; ++i) {
//     auto const it = set.insert(i);
//     REQUIRE(*it.first == i);
//     REQUIRE(!it.second);
//     REQUIRE(set.size() == 64);
//   }
//   std::vector<bool> v(64);
//   for (auto const i : set) {
//     v[i - 1] = true;
//   }
//   for (auto const b : v) {
//     REQUIRE(b);
//   }
// }
} // namespace util
} // namespace marlon