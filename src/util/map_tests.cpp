#include "map.h"

#include <random>
#include <unordered_map>

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("marlon::util::Map") {
  auto random_engine = std::mt19937{std::random_device{}()};
  auto random_distribution = std::uniform_int_distribution<>{};
  auto const max_size = 64;
  auto const block =
      make_unique_block(Map<int, int>::memory_requirement(max_size));
  auto marlon_map = Map<int, int>{};
  auto std_map = std::unordered_map<int, int>{};
  REQUIRE(marlon_map.begin() == marlon_map.cbegin());
  REQUIRE(marlon_map.end() == marlon_map.cend());
  REQUIRE(marlon_map.begin() == marlon_map.end());
  REQUIRE(marlon_map.begin() == marlon_map.cend());
  REQUIRE(marlon_map.cbegin() == marlon_map.end());
  REQUIRE(marlon_map.cbegin() == marlon_map.cend());
  REQUIRE(marlon_map.size() == 0);
  REQUIRE(marlon_map.max_size() == 0);
  REQUIRE(marlon_map.max_bucket_count() == 0);
  for (int i = 1; i <= max_size; ++i) {
    marlon_map = Map<int, int>{block.get(), static_cast<std::size_t>(i)};
    std_map = std::unordered_map<int, int>{};
    REQUIRE(marlon_map.begin() == marlon_map.cbegin());
    REQUIRE(marlon_map.end() == marlon_map.cend());
    REQUIRE(marlon_map.begin() == marlon_map.end());
    REQUIRE(marlon_map.begin() == marlon_map.cend());
    REQUIRE(marlon_map.cbegin() == marlon_map.end());
    REQUIRE(marlon_map.cbegin() == marlon_map.cend());
    REQUIRE(marlon_map.size() == 0);
    REQUIRE(marlon_map.max_size() >= static_cast<std::size_t>(i));
    REQUIRE(marlon_map.max_bucket_count() * marlon_map.max_load_factor() >=
            marlon_map.max_size());
    for (int j = 0; j < i; ++j) {
      auto const key = random_distribution(random_engine);
      auto const value = random_distribution(random_engine);
      REQUIRE(marlon_map.insert(std::pair{key, value}).second ==
              std_map.insert({key, value}).second);
      REQUIRE(marlon_map.insert(std::pair{key, value}).second ==
              std_map.insert({key, value}).second);
      REQUIRE(marlon_map.size() == std_map.size());
      for (auto const &p : marlon_map) {
        REQUIRE(marlon_map.at(p.first) == p.second);
      }
      for (auto const &p : std_map) {
        REQUIRE(marlon_map.at(p.first) == p.second);
      }
    }
    marlon_map.clear();
    std_map.clear();
    REQUIRE(marlon_map.size() == std_map.size());
    for (int j = 0; j < i; ++j) {
      auto const key = random_distribution(random_engine);
      auto const value = random_distribution(random_engine);
      REQUIRE(marlon_map.emplace(key, value).second ==
              std_map.emplace(key, value).second);
      REQUIRE(marlon_map.emplace(key, value).second ==
              std_map.emplace(key, value).second);
      REQUIRE(marlon_map.size() == std_map.size());
      for (auto const &p : marlon_map) {
        REQUIRE(marlon_map.at(p.first) == p.second);
      }
      for (auto const &p : std_map) {
        REQUIRE(marlon_map.at(p.first) == p.second);
      }
    }
    for (auto const &p : std_map) {
      REQUIRE(marlon_map.erase(p.first) == 1);
    }
    REQUIRE(marlon_map.size() == 0);
  }
}
} // namespace util
} // namespace marlon