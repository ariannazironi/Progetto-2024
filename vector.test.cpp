#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"
#include "doctest.h"

TEST_CASE("Testing operator +") {
  SUBCASE("Positive components") {
    const sim::Vector v1{1, 1};
    const sim::Vector v2{1, 1};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(2.0).epsilon(0.1));
  };

  SUBCASE("Negative components") {
    const sim::Vector v1{-2, -1};
    const sim::Vector v2{-3, -2};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() == doctest::Approx(-5.0).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(-3.0).epsilon(0.1));
  };

  SUBCASE("Null components") {
    const sim::Vector v1{3, 0};
    const sim::Vector v2{0, 5};
    const sim::Vector sum{v1 + v2};
    CHECK(sum.get_x() ==  doctest::Approx(3.0).epsilon(0.1));
    CHECK(sum.get_y() ==  doctest::Approx(5.0).epsilon(0.1));
  }; 
}

TEST_CASE("Testing operator +=") {
  SUBCASE("Positive components") {
    sim::Vector v1{2,3}; //non lo dichiaro const perchè l'operatore += lo modifica
    const sim::Vector v2{1,1};
    v1 += v2;
    CHECK(v1.get_x() == doctest::Approx(3.0).epsilon(0.1));
    CHECK(v1.get_y() == doctest::Approx(4.0).epsilon(0.1));
  };

  SUBCASE("Negative components") {
    sim::Vector v1{-5,-4};
    const sim::Vector v2{-2,-2};
    v1 += v2;
    CHECK(v1.get_x() == doctest::Approx(-7.0).epsilon(0.1));
    CHECK(v1.get_y() == doctest::Approx(-6.0).epsilon(0.1));
  };

  SUBCASE("Null components") {
    sim::Vector v1{3,0};
    const sim::Vector v2{0,1};
    v1 +=v2; 
    CHECK(v1.get_x() == doctest::Approx(3.0).epsilon(0.1));
    CHECK(v1.get_y() == doctest::Approx(1.0).epsilon(0.1));
  };
}

TEST_CASE("Testing operator -") {
  SUBCASE("Positive components") {
    const sim::Vector v1{2,4};
    const sim::Vector v2{1,1};
    const sim::Vector difference{v1 - v2};
    CHECK(difference.get_x() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(difference.get_y() == doctest::Approx(3.0).epsilon(0.1));
};

SUBCASE("Negative components") {
  const sim::Vector v1{-3,-2};
  const sim::Vector v2{-2,-1};
  const sim::Vector difference{v1-v2};
  CHECK(difference.get_x() == doctest::Approx(-1.0).epsilon(0.1));
  CHECK(difference.get_y() == doctest::Approx(-1.0).epsilon(0.1));
};

SUBCASE("Null components") {
  const sim::Vector v1{0,1};
  const sim::Vector v2{0,0};
  const sim::Vector difference{v1-v2};
  CHECK(difference.get_x() == doctest::Approx(0.).epsilon(0.1));
  CHECK(difference.get_y() == doctest::Approx(1.0).epsilon(0.1));
};
}

TEST_CASE("Testing distance") {
  SUBCASE("Positive components") {
    const sim::Vector v1{1,1};
    const sim::Vector v2{6,1};
    const float distance= v1.distance(v2);
    CHECK(distance == doctest::Approx(5.0).epsilon(0.1));
  };

   SUBCASE("Negative components") {
    const sim::Vector v1{-1,-1};
    const sim::Vector v2{-6,-1};
    const float distance= v1.distance(v2);
    CHECK(distance == doctest::Approx(5.0).epsilon(0.1));
  };

    SUBCASE("Null components") {
    const sim::Vector v1{-1,0};
    const sim::Vector v2{0,0};
    const float distance= v1.distance(v2);
    CHECK(distance == doctest::Approx(1.0).epsilon(0.1));
  };
}

TEST_CASE("Testing norm_vector") {
  SUBCASE("Positive components") {
    const sim::Vector v1{3,4};
    float norm = v1.norm_vector();
    CHECK(norm == doctest::Approx(5.0).epsilon(0.1));
  };

  SUBCASE("Negative components") {
    const sim::Vector v1{-4,-3};
    float norm = v1.norm_vector();
    CHECK(norm == doctest::Approx(5.0).epsilon(0.1));
  };

  SUBCASE("Null components") {
    const sim::Vector v1{0,0};
    float norm = v1.norm_vector();
    CHECK(norm == doctest::Approx(0.).epsilon(0.1));
  };
}

TEST_CASE("Testing operator!=") {
    const sim::Vector v1{2,3};
    const sim::Vector v2{2,4};
    CHECK(v1 != v2);
}

TEST_CASE("Testing operator ==") {
    const sim::Vector v1{2,3};
    const sim::Vector v2{2,3};
    CHECK(v1 == v2);
}

TEST_CASE("Testing operator*") {
  SUBCASE("Positive scalar") {
    const sim::Vector v1{1,0};
    const float f = 2;
    sim::Vector dot_scalar{v1*f};
    CHECK(dot_scalar.get_x() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(dot_scalar.get_y() == doctest::Approx(0.).epsilon(0.1));
  };

  SUBCASE("Negative scalar") {
    const sim::Vector v1{1,-2};
    const float f = -2;
    sim::Vector dot_scalar{v1*f};
    CHECK(dot_scalar.get_x() == doctest::Approx(-2.0).epsilon(0.1));
    CHECK(dot_scalar.get_y() == doctest::Approx(4).epsilon(0.1));
  };

  SUBCASE("Null scalar") {
    const sim::Vector v1{1,-2};
    const float f = 0;
    sim::Vector dot_scalar{v1*f};
    CHECK(dot_scalar.get_x() == doctest::Approx(0.).epsilon(0.1));
    CHECK(dot_scalar.get_y() == doctest::Approx(0.).epsilon(0.1));
  };
}

TEST_CASE("Testing product") {
  SUBCASE("Positive components") {
    const sim::Vector v1{1,1};
    const sim::Vector v2{2,2};
    float dot_product = v1.product(v2);
    CHECK(dot_product == doctest::Approx(4.0).epsilon(0.1));
  };

  SUBCASE("Negative components") {
    const sim::Vector v1{-1,-1};
    const sim::Vector v2{-2,-2};
    float dot_product = v1.product(v2);
    CHECK(dot_product == doctest::Approx(4.0).epsilon(0.1));
  };

  SUBCASE("Null components") {
    const sim::Vector v1{0,1};
    const sim::Vector v2{2,0};
    float dot_product = v1.product(v2);
    CHECK(dot_product == doctest::Approx(0.).epsilon(0.1));
  };
}
