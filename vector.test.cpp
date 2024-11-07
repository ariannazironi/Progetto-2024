#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"

#include "doctest.h"

TEST_CASE("Testing operator +") {
  SUBCASE("Positive components") {
    Vector v1{1, 1};
    Vector v2{1, 1};
    Vector sum{v1 + v2};
    CHECK(sum.get_x() == 2);
    CHECK(sum.get_y() == 2);
  };
}