#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"

#include "doctest.h"

TEST_CASE("Testing operator +") {
  SUBCASE("Positive components") {
    const sim::Vector v1{1, 1};
    const sim::Vector v2{1, 1};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() == 2);
    CHECK(sum.get_y() == 2);
  };

  SUBCASE("Negative components") {
    const sim::Vector v1{-2, -1};
    const sim::Vector v2{-3, -2};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() == -5);
    CHECK(sum.get_y() == -3);
  };

  SUBCASE("Null components") {
    const sim::Vector v1{3, 0};
    const sim::Vector v2{0, 5};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() == 3);
    CHECK(sum.get_y() == 5);
  };

  SUBCASE(""){

  };
}