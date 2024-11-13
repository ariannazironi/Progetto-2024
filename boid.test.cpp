//#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"

#include <vector>

#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid class") {
  const sim::Vector pos0{0., 0.};
  const sim::Vector vel0{0., 0.};

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

  SUBCASE("Testing getters") {
    CHECK(b0.get_pos().get_x() == 0.);
    CHECK(b0.get_pos().get_y() == 0.);
    CHECK(b0.get_vel().get_x() == 0.);
    CHECK(b0.get_vel().get_y() == 0.);

    CHECK(b1.get_pos().get_x() == 3.);
    CHECK(b1.get_pos().get_y() == 4.);
    CHECK(b1.get_vel().get_x() == 1.);
    CHECK(b1.get_vel().get_y() == 1.);
  }

  SUBCASE("Testing find_near method") {
    auto near = b0.find_near(boids, 5.);  // Trova i boid vicini
    CHECK(near.size() == 1);
    CHECK(near[0].get_pos() == pos3);
  }

  SUBCASE("Testing the separation method") {
    float s = 1.;
    float ds = 5.;
    auto near = b0.find_near(boids, 5.);
    sim::Vector separation_vector = b0.separation(s, ds, near);
    sim::Vector expected_vector{0., 0.};
    for (const auto& boid : near) {
      sim::Vector diff = boid.get_pos() - b0.get_pos();
      expected_vector += diff * (-s);
    };
    CHECK(separation_vector.get_x() ==
          doctest::Approx(expected_vector.get_x()));
    CHECK(separation_vector.get_y() ==
          doctest::Approx(expected_vector.get_y()));
  }
}