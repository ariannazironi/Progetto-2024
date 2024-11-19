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
  sim::Boid b1(pos1, vel1);
  sim::Boid b2(pos2, vel2);
  sim::Boid b3(pos3, vel3);

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
    const float a = 0.5f;
    sim::Vector alignment_vector = b0.alignment(a, near);
    CHECK(alignment_vector.get_x() == doctest::Approx(2.5f).epsilon(0.1));
    CHECK(alignment_vector.get_y() == doctest::Approx(1.25f).epsilon(0.01));

    sim::Vector alignment_vector_1 = b0.alignment(a, near_1);
    CHECK(alignment_vector_1.get_x() == doctest::Approx(1.0f).epsilon(0.1));
    CHECK(alignment_vector_1.get_y() ==
          doctest::Approx(0.875f).epsilon(0.001));

    sim::Vector alignment_vector_2 = b0.alignment(a, near_2);
    CHECK(alignment_vector_2.get_x() ==
          doctest::Approx(1.16f).epsilon(0.01));
    CHECK(alignment_vector_2.get_y() ==
          doctest::Approx(-0.416f).epsilon(0.01));
  }

  SUBCASE("Testing the separation method") {
    sim::Vector separation_vector = b0.separation(1.0f, 5.0f, near);

    CHECK(separation_vector.get_x() == doctest::Approx(-2.0f).epsilon(0.1));
    CHECK(separation_vector.get_y() == doctest::Approx(-2.0f).epsilon(0.1));

    sim::Vector separation_vector_1 = b0.separation(1.0f, 5.5f, near_1);

    CHECK(separation_vector_1.get_x() == doctest::Approx(-5.0f).epsilon(0.1));
    CHECK(separation_vector_1.get_y() == doctest::Approx(-6.0f).epsilon(0.1));

    sim::Vector separation_vector_2 = b0.separation(1.0f, 8.5f, near_2);

    CHECK(separation_vector_2.get_x() == doctest::Approx(-6.0f).epsilon(0.1));
    CHECK(separation_vector_2.get_y() == doctest::Approx(-14.0f).epsilon(0.1));
  }

  SUBCASE("Testing cohesion method") {
    const float c = 2.0f;

    sim::Vector choesion_vector = b0.cohesion(c, near);
    CHECK(choesion_vector.get_x() == doctest::Approx(4.0f).epsilon(0.1));
    CHECK(choesion_vector.get_y() == doctest::Approx(4.0f).epsilon(0.1));

    sim::Vector choesion_vector_1 = b0.cohesion(c, near_1);
    CHECK(choesion_vector_1.get_x() == doctest::Approx(5.0f).epsilon(0.1));
    CHECK(choesion_vector_1.get_y() == doctest::Approx(6.0f).epsilon(0.1));

    sim::Vector choesion_vector_2 = b0.cohesion(c, near_2);
    CHECK(choesion_vector_2.get_x() == doctest::Approx(4.0f).epsilon(0.1));
    CHECK(choesion_vector_2.get_y() == doctest::Approx(9.33f).epsilon(0.01));
  }

  SUBCASE("Testing limit_velocity method"){
    b2.limit_velocity(3.0f);

    CHECK(b2.get_vel().get_x() == doctest::Approx(2.2).epsilon(0.1));
    CHECK(b2.get_vel().get_y() == doctest::Approx(-3.0).epsilon(0.1));

    b3.limit_velocity(2.5f);
    CHECK(b3.get_vel().get_x() == doctest::Approx(3.5).epsilon(0.1));
    CHECK(b3.get_vel().get_y() == doctest::Approx(1.25).epsilon(0.01));
  }

  SUBCASE("Testing the set_vel and set_pos method") {
    sim::Vector new_vel = {5.0f ,4.0f};
    sim::Vector new_pos = {2.0f, 6.0f};
    b1.set_vel(new_vel);
    b1.set_pos(new_pos);
    CHECK(b1.get_vel().get_x() == doctest::Approx(5.0f).epsilon(0.1));
    CHECK(b1.get_vel().get_y() == doctest::Approx(4.0f).epsilon(0.01));
    CHECK(b1.get_pos().get_x() == doctest::Approx(2.0f).epsilon(0.1));
    CHECK(b1.get_pos().get_y() == doctest::Approx(6.0f).epsilon(0.01));
  }
}
