<<<<<<< HEAD
// #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
=======
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
>>>>>>> e556e43f818532ca1090edcee41003072f5d1483

#include "flock.hpp"

#include <vector>

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Flock class") {
  const sim::Vector pos0{0.f, 0.f};
  const sim::Vector vel0{2.f, 0.f};

  const sim::Vector pos1{3.f, 4.f};
  const sim::Vector vel1{1.f, 1.f};

  const sim::Vector pos2(1.f, 8.f);
  const sim::Vector vel2(5.f, -6.f);

  const sim::Vector pos3(2.f, 2.f);
  const sim::Vector vel3(7.f, 2.5f);

  sim::Boid b0(pos0, vel0);
  sim::Boid b1(pos1, vel1);
  sim::Boid b2(pos2, vel2);
  sim::Boid b3(pos3, vel3);

    const float closeness_parameter_ = 10.0f;

    const float distance_of_separation_ = 4.0f;

    const float separation_parameter_ = 0.1f;

    const float allignment_parameter_ = 0.2f;

    const float cohesion_parameter_ = 0.3f;

    const float max_speed_ = 10.0f;

    sim::Flock flock(closeness_parameter_, distance_of_separation_,
                     separation_parameter_, allignment_parameter_,
                     cohesion_parameter_, max_speed_);

    flock.add_boids(b0);
    flock.add_boids(b1);
    flock.add_boids(b2);
    flock.add_boids(b3);
<<<<<<< HEAD

    
    CHECK(flock.get_boids().size() == 4);

    CHECK(flock.find_centermass(b0).get_x() == doctest::Approx(2.0f));
    CHECK(flock.find_centermass(b0).get_y() ==
          doctest::Approx(4.667).epsilon(0.001));

    CHECK(flock.find_separation(b0).get_x() ==
          doctest::Approx(-0.2f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() == -0.2f);

    const float delta_t = 0.5f;

    flock.update_boids(delta_t);

    const auto& updated_boids = flock.get_boids();

    /* std::vector<sim::Boid> near =
         b0.find_near(flock.get_boids(), closeness_parameter_);

     CHECK(near.size() == 3);
     CHECK(near[0].get_pos() == pos1);*/


    CHECK(updated_boids[0].get_vel().get_x() ==
          doctest::Approx(2.8667).epsilon(0.0001));
    CHECK(updated_boids[0].get_vel().get_y() ==
          doctest::Approx(1.0333).epsilon(0.0001));
    CHECK(updated_boids[0].get_pos().get_x() ==
          doctest::Approx(1.4333).epsilon(0.0001));
    CHECK(updated_boids[0].get_pos().get_y() ==
          doctest::Approx(0.5167).epsilon(0.0001));
=======
 SUBCASE("Testing find methods") {

    CHECK(flock.get_boids().size() == 4);

    CHECK(flock.find_centermass(b0).get_x() == doctest::Approx(2.0f));
    CHECK(flock.find_centermass(b0).get_x() == doctest::Approx(2.0f));
    CHECK(flock.find_centermass(b0).get_y() ==
          doctest::Approx(4.667).epsilon(0.001));

    CHECK(flock.find_separation(b0).get_x() == doctest::Approx(- 0.2f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() == doctest::Approx(- 0.2f).epsilon(0.1) );

    CHECK(flock.find_alignment(b0).get_x() == doctest::Approx(0.466f).epsilon(0.001));
    CHECK(flock.find_alignment(b0).get_y() == doctest::Approx(- 0.166f).epsilon(0.001));

    CHECK(flock.find_cohesion(b0).get_x() == doctest::Approx(0.6f).epsilon(0.1));
    CHECK(flock.find_cohesion(b0).get_y() == doctest::Approx(1.4f).epsilon(0.1));

    CHECK(flock.find_offset(b0).get_x() == doctest::Approx(0.866f).epsilon(0.001));
    CHECK(flock.find_offset(b0).get_y() == doctest::Approx(1.033f).epsilon(0.001));
>>>>>>> e556e43f818532ca1090edcee41003072f5d1483
          
  }

  SUBCASE("Testing update method") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t);

    auto updated_boids = flock.get_boids();
    CHECK(updated_boids[0].get_vel().get_x() ==
          doctest::Approx(2.866).epsilon(0.001));
    CHECK(updated_boids[0].get_vel().get_y() ==
          doctest::Approx(1.033).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_x() ==
          doctest::Approx(1.4333).epsilon(0.0001));
    CHECK(updated_boids[0].get_pos().get_y() ==
          doctest::Approx(0.5166).epsilon(0.0001));
}
}
