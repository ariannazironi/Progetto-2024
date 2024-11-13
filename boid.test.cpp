//#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"

#include "doctest.h"

namespace sim {
TEST_CASE("Testing the boid class") {
    SUBCASE("Testing getter") {
        Vector pos(9., 5.);
        Vector vel(3., 2.);
        Boid boid(pos, vel);
        CHECK(boid.get_pos() == pos);
        CHECK(boid.get_vel() == vel);
    }

    SUBCASE("Testing the find_near method") {
        Vector pos1(0., 0.);
        Vector pos2(1., 1.);
        Vector pos3(5., 5.);

        Vector vel(1, 1);

        Boid boid1(pos1, vel);
        Boid boid2(pos2, vel);
        Boid boid3(pos3, vel);

        std::vector<sim::Boid> boids = {boid1, boid2, boid3};
        
        auto near_boids = boid1.find_near(boids, 2.0f);

        CHECK(near_boids.size() == 1);
        CHECK(near_boids[0].get_pos() == pos2);
    }
}
}