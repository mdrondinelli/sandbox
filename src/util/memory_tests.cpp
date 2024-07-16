#include "memory.h"

#include <memory>
#include <random>

#include <catch2/catch_test_macros.hpp>

namespace marlon {
namespace util {
TEST_CASE("Use align to align sizes to a power-of-two alignment.") {
  REQUIRE(align(10, 16) == 16);
  REQUIRE(align(4, 8) == 8);
  REQUIRE(align(4, 4) == 4);
}


// TEST_CASE("Stack_allocator usage") {
//   REQUIRE(Stack_allocator<8>::memory_requirement({2, 4, 8, 16}) == 40);
//   REQUIRE(Stack_allocator<16>::memory_requirement({2, 4, 8, 16}) == 64);
//   auto const allocator_block =
//       make_unique_block(Stack_allocator<8>::memory_requirement({2, 4, 8, 16}));
//   auto allocator = Stack_allocator<8>{allocator_block.get()};
//   REQUIRE_NOTHROW([&]() {
//     auto const a1 = make_unique_block(2, &allocator);
//     auto const a2 = make_unique_block(4, &allocator);
//     auto const a3 = make_unique_block(8, &allocator);
//     auto const a4 = make_unique_block(16, &allocator);
//     REQUIRE(ptrdiff(a2.begin(), a1.begin()) == 8);
//     REQUIRE(ptrdiff(a3.begin(), a2.begin()) == 8);
//     REQUIRE(ptrdiff(a4.begin(), a3.begin()) == 8);
//   }());
//   REQUIRE_THROWS([&]() {
//     auto a1 = make_unique_block(2, &allocator);
//     auto a2 = make_unique_block(4, &allocator);
//     auto a3 = make_unique_block(8, &allocator);
//     auto const a4 = make_unique_block(16, &allocator);
//     REQUIRE(ptrdiff(a2.begin(), a1.begin()) == 8);
//     REQUIRE(ptrdiff(a3.begin(), a2.begin()) == 8);
//     REQUIRE(ptrdiff(a4.begin(), a3.begin()) == 8);
//     allocator.free(a1.release());
//     allocator.free(a2.release());
//     allocator.free(a3.release());
//     auto const a5 = make_unique_block(1, &allocator);
//   }());
// }

// TEST_CASE("Free_list_allocator usage") {
//   auto const allocator_block = make_unique_block(4096);
//   auto allocator = Free_list_allocator<Stack_allocator<8>, 1, 8>{
//       Stack_allocator<8>{allocator_block.get()}};
//   auto rng = std::mt19937_64{};
//   auto size_distribution = std::uniform_int_distribution<std::size_t>{1, 8};
//   REQUIRE_NOTHROW([&]() {
//     for (auto i = 0; i < 3; ++i) {
//       auto blocks = std::vector<Block>{};
//       for (auto j = 0; j < 4096 / 8; ++j) {
//         blocks.emplace_back(allocator.alloc(size_distribution(rng)));
//       }
//       for (auto j = 0; j < 4096 / 8; ++j) {
//         auto index_distribution =
//             std::uniform_int_distribution<std::size_t>{0, blocks.size() - 1};
//         auto const index = index_distribution(rng);
//         auto const it = blocks.begin() + index;
//         allocator.free(*it);
//         blocks.erase(it);
//       }
//     }
//   }());
// }
} // namespace util
} // namespace marlon