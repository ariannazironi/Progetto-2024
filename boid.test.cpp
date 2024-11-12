#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"
#include "vector.hpp"
#include <vector>

#include "doctest.h"

TEST_CASE("Testing Boid class") {
const sim::Vector pos0{0.,0.};
const sim::Vector vel0{0.,0.};

const sim::Vector pos1{3., 4.};
const sim::Vector vel1{1.,1.};

const sim::Vector pos2(1., 8.);
const sim::Vector vel2(5., -6.);

const sim::Vector pos3(2., 2.);
const sim::Vector vel3(7., 2.5);

const sim::Boid b0(pos0, vel0);
const sim::Boid b1(pos1, vel1);
const sim::Boid b2(pos2, vel2);
const sim::Boid b3(pos3, vel3);

const std::vector<sim::Boid> boids{b1, b2, b3};

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
auto near = b0.find_near(boids, 5.); // Trova i boid vicini
CHECK(near.size() == 1); 
}
}