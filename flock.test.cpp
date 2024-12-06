#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"
#include <vector>
#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing three close boids") {
  const sim::Vector pos0{0.f, 0.f};
  const sim::Vector vel0{2.f, 0.f};

  const sim::Vector pos1{3.f, 4.f};
  const sim::Vector vel1{1.f, 1.f};

  const sim::Vector pos2(1.f, 8.f);
  const sim::Vector vel2(5.f, -6.f);

  const sim::Vector pos3(2.f, 2.f);
  const sim::Vector vel3(7.f, 2.5f);

  sim::Boid b0(pos0, vel0, 100.f);
  sim::Boid b1(pos1, vel1, 100.f);
  sim::Boid b2(pos2, vel2, 100.f );
  sim::Boid b3(pos3, vel3, 100.f);

  const float closeness_parameter_ = 10.0f;

  const float distance_of_separation_ = 4.0f;

  const float separation_parameter_ = 0.1f;

  const float allignment_parameter_ = 0.2f;

  const float cohesion_parameter_ = 0.3f;

  const float max_speed_ = 10.0f;

  const float min_speed_ = 1.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_boids(b3);

  SUBCASE("Testing getter method:"){
    auto boids = flock.get_boids();

    CHECK(boids.size() == 4);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
    CHECK(boids[3] == b3);
   }

  SUBCASE("Testing find methods:") {

    CHECK(flock.find_separation(b0).get_x() == doctest::Approx(-0.2f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() == doctest::Approx(-0.2f).epsilon(0.1));

    CHECK(flock.find_alignment(b0).get_x() == doctest::Approx(0.466f).epsilon(0.001));
    CHECK(flock.find_alignment(b0).get_y() == doctest::Approx(-0.166f).epsilon(0.001));

    CHECK(flock.find_cohesion(b0).get_x() == doctest::Approx(0.6f).epsilon(0.1));
    CHECK(flock.find_cohesion(b0).get_y() == doctest::Approx(1.4f).epsilon(0.1));

    CHECK(flock.find_deltav(b0).get_x() == doctest::Approx(0.866f).epsilon(0.001));
    CHECK(flock.find_deltav(b0).get_y() == doctest::Approx(1.033f).epsilon(0.001));
  }

  SUBCASE("Testing update method:") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t,300.f,300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[0].get_vel().get_x() == doctest::Approx(2.866).epsilon(0.001));
    CHECK(updated_boids[0].get_vel().get_y() == doctest::Approx(1.033).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_x() == doctest::Approx(1.4333).epsilon(0.0001));
    CHECK(updated_boids[0].get_pos().get_y() == doctest::Approx(0.5166).epsilon(0.0001));
  }
}

TEST_CASE("Testing no close boids") {
  const sim::Vector pos0{0.f, 0.f};
  const sim::Vector vel0{2.f, 0.f};

  const sim::Vector pos1{4.f, 3.7f};
  const sim::Vector vel1{1.f, 1.f};

  const sim::Vector pos2(1.5f, 7.f);
  const sim::Vector vel2(4.f, -3.f);

  sim::Boid b0(pos0, vel0, 100.f);
  sim::Boid b1(pos1, vel1, 100.f);
  sim::Boid b2(pos2, vel2, 100.f );

  const float closeness_parameter_ = 1.0f;

  const float distance_of_separation_ = 4.0f;

  const float separation_parameter_ = 0.1f;

  const float allignment_parameter_ = 0.2f;

  const float cohesion_parameter_ = 0.3f;

  const float max_speed_ = 10.0f;

  const float min_speed_ = 0.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);

  SUBCASE("Testing getter method:"){
    auto boids = flock.get_boids();

    CHECK(boids.size() == 3);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
   }

  SUBCASE("Testing find methods:") {

    CHECK(flock.find_separation(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_alignment(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_alignment(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_cohesion(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_cohesion(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_deltav(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_deltav(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));
  }

  SUBCASE("Testing update method:") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t,300.f,300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[0].get_vel().get_x() == doctest::Approx(2.f).epsilon(0.001));
    CHECK(updated_boids[0].get_vel().get_y() == doctest::Approx(0.f).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_x() == doctest::Approx(1.f).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_y() == doctest::Approx(0.f).epsilon(0.001));

    CHECK(updated_boids[1].get_vel().get_x() == doctest::Approx(1.f).epsilon(0.001));
    CHECK(updated_boids[1].get_vel().get_y() == doctest::Approx(1.f).epsilon(0.001));
    CHECK(updated_boids[1].get_pos().get_x() == doctest::Approx(4.5f).epsilon(0.001));
    CHECK(updated_boids[1].get_pos().get_y() == doctest::Approx(4.2f).epsilon(0.001));

    CHECK(updated_boids[2].get_vel().get_x() == doctest::Approx(4.f).epsilon(0.001));
    CHECK(updated_boids[2].get_vel().get_y() == doctest::Approx(-3.f).epsilon(0.001));
    CHECK(updated_boids[2].get_pos().get_x() == doctest::Approx(3.5f).epsilon(0.001));
    CHECK(updated_boids[2].get_pos().get_y() == doctest::Approx(5.5f).epsilon(0.001));
  }
}

TEST_CASE("Testing one close boids") {
  const sim::Vector pos0{2.f, 2.f};
  const sim::Vector vel0{1.f, 1.f};

  const sim::Vector pos1{-3.f, -4.f};
  const sim::Vector vel1{1.f, 1.f};

  const sim::Vector pos2(1.f, 8.f);
  const sim::Vector vel2(11.f, -6.f);

  const sim::Vector pos3(0.f, 3.f);
  const sim::Vector vel3(5.f, 0.f);

  sim::Boid b0(pos0, vel0, 100.f );
  sim::Boid b1(pos1, vel1, 100.f);
  sim::Boid b2(pos2, vel2, 100.f);
  sim::Boid b3(pos3, vel3, 180.f);

  const float closeness_parameter_ = 3.0f;

  const float distance_of_separation_ = 4.0f;

  const float separation_parameter_ = 0.1f;

  const float allignment_parameter_ = 0.2f;

  const float cohesion_parameter_ = 0.3f;

  const float max_speed_ = 10.0f;

  const float min_speed_ = 1.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_boids(b3);

  SUBCASE("Testing getter method:"){
    auto boids = flock.get_boids();

    CHECK(boids.size() == 4);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
    CHECK(boids[3] == b3);
   }

  SUBCASE("Testing find methods:") {

    CHECK(flock.find_separation(b3).get_x() == doctest::Approx(-0.2f).epsilon(0.1));
    CHECK(flock.find_separation(b3).get_y() == doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_alignment(b3).get_x() == doctest::Approx(-0.8f).epsilon(0.1));
    CHECK(flock.find_alignment(b3).get_y() == doctest::Approx(0.2f).epsilon(0.1));

    CHECK(flock.find_cohesion(b3).get_x() == doctest::Approx(0.6f).epsilon(0.1));
    CHECK(flock.find_cohesion(b3).get_y() == doctest::Approx(-0.3f).epsilon(0.1));

    CHECK(flock.find_deltav(b3).get_x() == doctest::Approx(-0.4f).epsilon(0.1));
    CHECK(flock.find_deltav(b3).get_y() == doctest::Approx(0.1f).epsilon(0.1));
  }

  SUBCASE("Testing update method:") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t,300.f,300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[3].get_vel().get_x() == doctest::Approx(4.6f).epsilon(0.1));
    CHECK(updated_boids[3].get_vel().get_y() == doctest::Approx(0.1f).epsilon(0.1));
    CHECK(updated_boids[3].get_pos().get_x() == doctest::Approx(2.3f).epsilon(0.1));
    CHECK(updated_boids[3].get_pos().get_y() == doctest::Approx(3.05f).epsilon(0.01));
  }
}

TEST_CASE("Testing predators") {
  const sim::Vector pos0{2.f, 2.f};
  const sim::Vector vel0{1.f, 1.f};

  const sim::Vector pos1{-3.f, -4.f};
  const sim::Vector vel1{1.f, 1.f};

  const sim::Vector pos2(-4.f, 1.f);
  const sim::Vector vel2(11.f, -6.f);

  const sim::Vector pos3(0.f, 3.f);
  const sim::Vector vel3(5.f, 0.f);

  sim::Boid b1(pos0, vel0, 100.f );
  sim::Boid b2(pos1, vel1, 100.f);
  sim::Boid p1(pos2, vel2, 100.f);
  sim::Boid p2(pos3, vel3, 180.f);

  const float closeness_parameter_ = 3.0f;

  const float distance_of_separation_ = 4.0f;

  const float separation_parameter_ = 0.1f;

  const float allignment_parameter_ = 0.2f;

  const float cohesion_parameter_ = 0.3f;

  const float max_speed_ = 10.0f;

  const float min_speed_ = 1.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_predators(p1);
  flock.add_predators(p2);

   SUBCASE("Testing getter method:"){
    auto predators = flock.get_predators();

    CHECK(predators.size() == 2);
    CHECK(predators[0] == p1);
    CHECK(predators[1] == p2);
   }

  SUBCASE("Testing fid_prey method:"){
    const sim::Boid prey_1 = flock.find_prey(p1);
    const sim::Boid prey_2 = flock.find_prey(p2);
    CHECK(prey_1 == b2);
    CHECK(prey_2 == b1);

  }
}

TEST_CASE("Testing state method"){
  SUBCASE("State with three boids:") {
    const sim::Vector pos1{2.f, 3.f};
    const sim::Vector vel1{5.f, 6.f};
    const sim::Vector pos2{4.f, 5.f};
    const sim::Vector vel2{6.f, 7.f};
    const sim::Vector pos3{8.f, 9.f};
    const sim::Vector vel3{10.f, 11.f};

    sim::Boid b1{pos1, vel1, 50.f};
    sim::Boid b2{pos2, vel2, 50.f};
    sim::Boid b3{pos3, vel3, 50.f};

    sim::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.3f, 8.0f, 3.0f};
    flock.add_boids(b1);
    flock.add_boids(b2);
    flock.add_boids(b3);

    sim::Statistics state = flock.state();
    CHECK(state.mean_dist == doctest::Approx(5.66f).epsilon(0.01));
    CHECK(state.dev_dist == doctest::Approx(2.31f).epsilon(0.01));
    CHECK(state.mean_speed == doctest::Approx(10.63f).epsilon(0.01));
    CHECK(state.dev_speed == doctest::Approx(3.05f).epsilon(0.01));
  }

SUBCASE("State with two boids:") {
    const sim::Vector pos1{1.5f, 3.f};
    const sim::Vector vel1{2.f, 2.f};
    const sim::Vector pos2{4.f, 3.f};
    const sim::Vector vel2{3.f, 5.f};

    sim::Boid b1{pos1, vel1, 50.f};
    sim::Boid b2{pos2, vel2, 50.f};

    sim::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.3f, 8.0f, 3.0f};
    flock.add_boids(b1);
    flock.add_boids(b2);

    sim::Statistics state = flock.state();
    CHECK(state.mean_dist == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.dev_dist == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.mean_speed == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.dev_speed == doctest::Approx(0.f).epsilon(0.01));
}

}