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
}