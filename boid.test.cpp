#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"

#include <vector>

#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid class") {
  const sim::Vector pos0{0., 0.};
  const sim::Vector vel0{2., 0.};

  const sim::Vector pos1{3., 4.};
  const sim::Vector vel1{1., 1.};

  const sim::Vector pos2(1., 8.);
  const sim::Vector vel2(5., -6.);

  const sim::Vector pos3(2., 2.);
  const sim::Vector vel3(7., 2.5);

  const sim::Boid b0(pos0, vel0);
  const sim::Boid b1(pos1, vel1);
  const sim::Boid b2(pos2, vel2);
  const sim::Boid b3(pos3, vel3);

  const std::vector<sim::Boid> boids{b0, b1, b2, b3};
  auto near = b0.find_near(boids, 5.);
  auto near_1 = b0.find_near(boids, 6.);
  auto near_2 = b0.find_near(boids, 9.);

  SUBCASE("Testing getters") {
    CHECK(b0.get_pos().get_x() == 0.);
    CHECK(b0.get_pos().get_y() == 0.);
    CHECK(b0.get_vel().get_x() == 2.);
    CHECK(b0.get_vel().get_y() == 0.);

    CHECK(b1.get_pos().get_x() == 3.);
    CHECK(b1.get_pos().get_y() == 4.);
    CHECK(b1.get_vel().get_x() == 1.);
    CHECK(b1.get_vel().get_y() == 1.);
  }

  SUBCASE("Testing find_near method") {
    CHECK(near.size() == 1);
    CHECK(near[0].get_pos() == pos3);
    CHECK(near_1.size() == 2);
    CHECK(near_2.size() == 3);
  }

  SUBCASE("Testing alignment method") {
    const float a= 0.5;
  
    CHECK(b0.alignment(a, near).get_x() == doctest::Approx(2.5).epsilon(0.1));
    CHECK(b0.alignment(a, near).get_y() == doctest::Approx(1.25).epsilon(0.1));

    CHECK(b0.alignment(a, near_1).get_x() == doctest::Approx(1).epsilon(0.1));
    CHECK(b0.alignment(a, near_1).get_y() == doctest::Approx(0.875).epsilon(0.001));

    CHECK(b0.alignment(a, near_2).get_x() == doctest::Approx(1.166).epsilon(0.001));
    CHECK(b0.alignment(a, near_2).get_y() == doctest::Approx(-0.4161).epsilon(0.001));
  }

  SUBCASE("Testing the separation method") {
    float s = 1.0f;
    float ds = 5.0f;

    CHECK(b0.separation(s,ds,near).get_x() == doctest::Approx(-2));
    CHECK(b0.separation(s,ds,near).get_y() == doctest::Approx(-2));
  
    CHECK(b0.separation(s,ds,near_1).get_x() == doctest::Approx(-2));
    CHECK(b0.separation(s,ds,near_1).get_x()== doctest::Approx(-2));

  CHECK(b0.separation(s,ds,near_2).get_x() == doctest::Approx(-2));
  CHECK(b0.separation(s,ds,near_2).get_y()== doctest::Approx(-2));
}

SUBCASE("Testing cohesion method") {
  const float c = 2.0f; 
 CHECK(b0.cohesion(c, near).get_x() == doctest::Approx(4).epsilon(0.1));
 CHECK(b0.cohesion(c, near).get_y() == doctest::Approx(4).epsilon(0.1));
 CHECK(b0.cohesion(c, near_1).get_x() == doctest::Approx(5).epsilon(0.1));
 CHECK(b0.cohesion(c, near_1).get_y() == doctest::Approx(6).epsilon(0.1));
 CHECK(b0.cohesion(c, near_2).get_x() == doctest::Approx(4).epsilon(0.1));
 CHECK(b0.cohesion(c, near_2).get_y() == doctest::Approx(9.33).epsilon(0.01));
}

}
