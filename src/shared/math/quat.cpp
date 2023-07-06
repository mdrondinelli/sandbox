#include "quat.h"

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace math {
TEST_CASE("Quaternions can be constructed.") {
  const Quatf p{4.0f, {1.0f, 2.0f, 3.0f}};
  REQUIRE(p.w == 4.0f);
  REQUIRE(p.v == Vec3f{1.0f, 2.0f, 3.0f});
  const auto q = Quatf::identity();
  REQUIRE(q.w == 1.0f);
  REQUIRE(q.v == Vec3f::zero());
}

TEST_CASE("Quaternions can be compared.") {
  const Quatf p{1.0f, {1.0f, 0.0f, 0.0f}};
  const Quatf q{1.0f, {0.0f, 0.0f, 0.0f}};
  REQUIRE(p == p);
  REQUIRE(p != q);
}

TEST_CASE("Quaternions can be negated.") {
  const Quatf q{1.0f, {2.0f, 3.0f, 4.0f}};
  const Quatf minus_q{-1.0f, {-2.0f, -3.0f, -4.0f}};
  REQUIRE(-q == minus_q);
}

TEST_CASE("Quaternions can be multiplied") {
  const auto p = Quatf::identity();
  const auto q = Quatf::identity();
  REQUIRE(p * q == Quatf::identity());
}
} // namespace math
} // namespace marlon