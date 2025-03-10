#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"

#include <vector>

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing three close boids") {
  const sim::Vector pos0{0.f, 10.f};
  const sim::Vector vel0{30.f, 0.f};

  const sim::Vector pos1{30.f, 40.f};
  const sim::Vector vel1{50.f, 10.f};

  const sim::Vector pos2(10.f, 80.f);
  const sim::Vector vel2(50.f, -60.f);

  const sim::Vector pos3(40.f, 20.f);
  const sim::Vector vel3(70.f, 2.0f);

  sim::Boid b0(pos0, vel0, 180.f);
  sim::Boid b1(pos1, vel1, 180.f);
  sim::Boid b2(pos2, vel2, 180.f);
  sim::Boid b3(pos3, vel3, 180.f);

  const float closeness_parameter_ = 100.0f;

  const float distance_of_separation_ = 42.0f;

  const float separation_parameter_ = 0.3f;

  const float alignment_parameter_ = 0.4f;

  const float cohesion_parameter_ = 0.0002f;

  const float max_speed_ = 100.0f;

  const float min_speed_ = 25.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, alignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_boids(b3);

  SUBCASE("Testing getter method") {
    auto boids = flock.get_boids();
    auto near0 = b0.find_near(boids, closeness_parameter_);

    CHECK(boids.size() == 4);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
    CHECK(boids[3] == b3);
    CHECK(near0.size() == 3);
  }

  SUBCASE("Testing find methods") {
    CHECK(flock.find_separation(b0).get_x() ==
          doctest::Approx(-12.0f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() ==
          doctest::Approx(-3.0f).epsilon(0.1));

    CHECK(flock.find_alignment(b0).get_x() ==
          doctest::Approx(10.667f).epsilon(0.001));
    CHECK(flock.find_alignment(b0).get_y() ==
          doctest::Approx(-6.400f).epsilon(0.001));

    CHECK(flock.find_cohesion(b0).get_x() ==
          doctest::Approx(0.0053f).epsilon(0.0001));
    CHECK(flock.find_cohesion(b0).get_y() ==
          doctest::Approx(0.0073f).epsilon(0.0001));

    CHECK(flock.find_deltav(b0).get_x() ==
          doctest::Approx(-1.328f).epsilon(0.001));
    CHECK(flock.find_deltav(b0).get_y() ==
          doctest::Approx(-9.393f).epsilon(0.001));
  }

  SUBCASE("Testing update method") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t, 300.f, 300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[0].get_vel().get_x() ==
          doctest::Approx(28.672).epsilon(0.001));
    CHECK(updated_boids[0].get_vel().get_y() ==
          doctest::Approx(-9.393).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_x() ==
          doctest::Approx(14.336).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_y() ==
          doctest::Approx(5.304).epsilon(0.001));
  }
}

TEST_CASE("Testing no close boids") {
  const sim::Vector pos0{0.f, 10.f};
  const sim::Vector vel0{30.f, 0.f};

  const sim::Vector pos1{100.f, 87.f};
  const sim::Vector vel1{10.f, 70.f};

  const sim::Vector pos2(200.f, 300.f);
  const sim::Vector vel2(40.f, -30.f);

  sim::Boid b0(pos0, vel0, 180.f);
  sim::Boid b1(pos1, vel1, 180.f);
  sim::Boid b2(pos2, vel2, 180.f);

  const float closeness_parameter_ = 60.0f;

  const float distance_of_separation_ = 40.0f;

  const float separation_parameter_ = 0.3f;

  const float allignment_parameter_ = 0.4f;

  const float cohesion_parameter_ = 0.0003f;

  const float max_speed_ = 100.0f;

  const float min_speed_ = 30.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);

  SUBCASE("Testing getter method") {
    auto boids = flock.get_boids();
    auto near0 = b0.find_near(boids, closeness_parameter_);

    CHECK(boids.size() == 3);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
  }

  SUBCASE("Testing find methods") {
    CHECK(flock.find_separation(b0).get_x() ==
          doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() ==
          doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_alignment(b0).get_x() ==
          doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_alignment(b0).get_y() ==
          doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_cohesion(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_cohesion(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));

    CHECK(flock.find_deltav(b0).get_x() == doctest::Approx(0.f).epsilon(0.1));
    CHECK(flock.find_deltav(b0).get_y() == doctest::Approx(0.f).epsilon(0.1));
  }

  SUBCASE("Testing update method") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t, 300.f, 300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[0].get_vel().get_x() ==
          doctest::Approx(30.0f).epsilon(0.1));
    CHECK(updated_boids[0].get_vel().get_y() ==
          doctest::Approx(0.0f).epsilon(0.1));
    CHECK(updated_boids[0].get_pos().get_x() ==
          doctest::Approx(15.0f).epsilon(0.1));
    CHECK(updated_boids[0].get_pos().get_y() ==
          doctest::Approx(10.0f).epsilon(0.1));

    CHECK(updated_boids[1].get_vel().get_x() ==
          doctest::Approx(10.0f).epsilon(0.1));
    CHECK(updated_boids[1].get_vel().get_y() ==
          doctest::Approx(70.0f).epsilon(0.1));
    CHECK(updated_boids[1].get_pos().get_x() ==
          doctest::Approx(105.0f).epsilon(0.1));
    CHECK(updated_boids[1].get_pos().get_y() ==
          doctest::Approx(122.0f).epsilon(0.1));

    CHECK(updated_boids[2].get_vel().get_x() ==
          doctest::Approx(40.0f).epsilon(0.1));
    CHECK(updated_boids[2].get_vel().get_y() ==
          doctest::Approx(-30.0f).epsilon(0.1));
    CHECK(updated_boids[2].get_pos().get_x() ==
          doctest::Approx(220.0f).epsilon(0.1));
    CHECK(updated_boids[2].get_pos().get_y() ==
          doctest::Approx(285.0f).epsilon(0.1));
  }
}

TEST_CASE("Testing one close boids") {
  const sim::Vector pos0{20.f, 20.f};
  const sim::Vector vel0{25.f, 10.f};

  const sim::Vector pos1{45.f, 60.f};
  const sim::Vector vel1{15.f, 30.f};

  const sim::Vector pos2(100.f, 90.f);
  const sim::Vector vel2(50.f, -60.f);

  const sim::Vector pos3(150.f, 300.f);
  const sim::Vector vel3(50.f, 0.f);

  sim::Boid b0(pos0, vel0, 180.f);
  sim::Boid b1(pos1, vel1, 180.f);
  sim::Boid b2(pos2, vel2, 180.f);
  sim::Boid b3(pos3, vel3, 180.f);

  const float closeness_parameter_ = 100.0f;

  const float distance_of_separation_ = 50.0f;

  const float separation_parameter_ = 0.3f;

  const float allignment_parameter_ = 0.4f;

  const float cohesion_parameter_ = 0.0003f;

  const float max_speed_ = 100.0f;

  const float min_speed_ = 10.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b0);
  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_boids(b3);

  SUBCASE("Testing getter method") {
    auto boids = flock.get_boids();
    auto near0 = b0.find_near(boids, closeness_parameter_);

    CHECK(boids.size() == 4);
    CHECK(boids[0] == b0);
    CHECK(boids[1] == b1);
    CHECK(boids[2] == b2);
    CHECK(boids[3] == b3);
    CHECK(near0.size() == 1);
  }

  SUBCASE("Testing find methods") {
    CHECK(flock.find_separation(b0).get_x() ==
          doctest::Approx(-7.5f).epsilon(0.1));
    CHECK(flock.find_separation(b0).get_y() ==
          doctest::Approx(-12.0f).epsilon(0.1));

    CHECK(flock.find_alignment(b0).get_x() ==
          doctest::Approx(-4.0f).epsilon(0.1));
    CHECK(flock.find_alignment(b0).get_y() ==
          doctest::Approx(8.0f).epsilon(0.1));

    CHECK(flock.find_cohesion(b0).get_x() ==
          doctest::Approx(0.0075f).epsilon(0.0001));
    CHECK(flock.find_cohesion(b0).get_y() ==
          doctest::Approx(0.012f).epsilon(0.001));

    CHECK(flock.find_deltav(b0).get_x() ==
          doctest::Approx(-11.493f).epsilon(0.001));
    CHECK(flock.find_deltav(b0).get_y() ==
          doctest::Approx(-3.988f).epsilon(0.001));
  }

  SUBCASE("Testing update method") {
    const float delta_t = 0.5f;
    flock.update_boids(delta_t, 300.f, 300.f);
    auto updated_boids = flock.get_boids();

    CHECK(updated_boids[0].get_vel().get_x() ==
          doctest::Approx(13.507f).epsilon(0.001));
    CHECK(updated_boids[0].get_vel().get_y() ==
          doctest::Approx(6.012f).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_x() ==
          doctest::Approx(26.754f).epsilon(0.001));
    CHECK(updated_boids[0].get_pos().get_y() ==
          doctest::Approx(23.006f).epsilon(0.001));
  }
}

TEST_CASE("Testing predators") {
  const sim::Vector pos0{200.f, 150.f};
  const sim::Vector vel0{50.f, 15.f};

  const sim::Vector pos1{80.f, 20.f};
  const sim::Vector vel1{-10.f, 75.f};

  const sim::Vector pos2(150.f, 10.f);
  const sim::Vector vel2(-2.0f, -35.f);

  const sim::Vector pos3(100.f, 200.f);
  const sim::Vector vel3(50.f, 5.f);

  sim::Boid b1(pos0, vel0, 180.f);
  sim::Boid b2(pos1, vel1, 180.f);
  sim::Boid p1(pos2, vel2, 180.f);
  sim::Boid p2(pos3, vel3, 180.f);

  const float closeness_parameter_ = 100.0f;

  const float distance_of_separation_ = 40.0f;

  const float separation_parameter_ = 0.4f;

  const float allignment_parameter_ = 0.5f;

  const float cohesion_parameter_ = 0.0002f;

  const float max_speed_ = 100.0f;

  const float min_speed_ = 30.0f;

  sim::Flock flock(closeness_parameter_, distance_of_separation_,
                   separation_parameter_, allignment_parameter_,
                   cohesion_parameter_, max_speed_, min_speed_);

  flock.add_boids(b1);
  flock.add_boids(b2);
  flock.add_predators(p1);
  flock.add_predators(p2);

  SUBCASE("Testing getter method") {
    auto predators = flock.get_predators();

    CHECK(predators.size() == 2);
    CHECK(predators[0] == p1);
    CHECK(predators[1] == p2);

    auto boids = flock.get_boids();

    CHECK(boids.size() == 2);
    CHECK(boids[0] == b1);
    CHECK(boids[1] == b2);
  }

  SUBCASE("Testing find_prey method") {
    const sim::Boid prey_1 = flock.find_prey(p1);
    const sim::Boid prey_2 = flock.find_prey(p2);
    CHECK(prey_1 == b2);
    CHECK(prey_2 == b1);
  }

  SUBCASE("Testing update method") {
    const float delta_t = 0.5f;
    flock.update_predator(delta_t, 300.f, 300.f);
    auto updated_predators = flock.get_predators();

    CHECK(updated_predators[0].get_vel().get_x() ==
          doctest::Approx(-30.0f).epsilon(0.1));
    CHECK(updated_predators[0].get_vel().get_y() ==
          doctest::Approx(-31.0f).epsilon(0.1));
    CHECK(updated_predators[0].get_pos().get_x() ==
          doctest::Approx(135.0f).epsilon(0.1));
    CHECK(updated_predators[0].get_pos().get_y() ==
          doctest::Approx(300.0f).epsilon(0.01));

    CHECK(updated_predators[1].get_vel().get_x() ==
          doctest::Approx(90.0f).epsilon(0.01));
    CHECK(updated_predators[1].get_vel().get_y() ==
          doctest::Approx(-15.0f).epsilon(0.01));
    CHECK(updated_predators[1].get_pos().get_x() ==
          doctest::Approx(145.0f).epsilon(0.01));
    CHECK(updated_predators[1].get_pos().get_y() ==
          doctest::Approx(192.5f).epsilon(0.01));
  }
}

TEST_CASE("Testing state method") {
  SUBCASE("State() with three boids") {
    const sim::Vector pos1{20.f, 35.f};
    const sim::Vector vel1{50.f, 60.f};
    const sim::Vector pos2{40.f, 50.f};
    const sim::Vector vel2{60.f, 7.f};
    const sim::Vector pos3{80.f, 150.f};
    const sim::Vector vel3{10.f, 40.f};

    sim::Boid b1{pos1, vel1, 180.f};
    sim::Boid b2{pos2, vel2, 180.f};
    sim::Boid b3{pos3, vel3, 180.f};

    sim::Flock flock{100.f, 30.f, 0.4f, 0.5f, 0.0003f, 100.0f, 30.0f};
    flock.add_boids(b1);
    flock.add_boids(b2);
    flock.add_boids(b3);

    sim::Statistics state = flock.state();
    CHECK(state.mean_dist == doctest::Approx(87.47f).epsilon(0.01));
    CHECK(state.dev_dist == doctest::Approx(45.08f).epsilon(0.01));
    CHECK(state.mean_speed == doctest::Approx(59.91f).epsilon(0.01));
    CHECK(state.dev_speed == doctest::Approx(15.06f).epsilon(0.01));
  }

  SUBCASE("State() with two boids") {
    const sim::Vector pos1{150.f, 30.f};
    const sim::Vector vel1{20.f, 45.f};
    const sim::Vector pos2{120.f, 70.f};
    const sim::Vector vel2{30.f, -50.f};

    sim::Boid b1{pos1, vel1, 180.f};
    sim::Boid b2{pos2, vel2, 180.f};

    sim::Flock flock{100.f, 30.f, 0.4f, 0.5f, 0.0003f, 100.0f, 30.0f};
    flock.add_boids(b1);
    flock.add_boids(b2);

    sim::Statistics state = flock.state();
    CHECK(state.mean_dist == doctest::Approx(50.0f).epsilon(0.1));
    CHECK(state.dev_dist == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.mean_speed == doctest::Approx(53.78f).epsilon(0.01));
    CHECK(state.dev_speed == doctest::Approx(4.48f).epsilon(0.01));
  }

  SUBCASE("State() with one boid") {
    const sim::Vector pos1{48.f, 200.f};
    const sim::Vector vel1{20.f, -65.f};

    sim::Boid b1{pos1, vel1, 180.f};
    sim::Flock flock{100.f, 30.f, 0.4f, 0.5f, 0.0003f, 100.0f, 30.0f};
    flock.add_boids(b1);
    sim::Statistics state = flock.state();
    CHECK(state.mean_dist == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.dev_dist == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.mean_speed == doctest::Approx(0.f).epsilon(0.01));
    CHECK(state.dev_speed == doctest::Approx(0.f).epsilon(0.01));
  }
}