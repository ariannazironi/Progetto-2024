#include "flock.hpp"

#include <cassert>
#include <cmath>
#include <numeric>
#include <vector>

namespace sim {

Flock::Flock(const float distance, const float ds_parameter,
             const float s_parameter, const float a_parameter,
             const float c_parameter, const float max_speed)
    : closeness_parameter_(distance),
      distance_of_separation_(ds_parameter),
      separation_parameter_(s_parameter),
      allignment_parameter_(a_parameter),
      cohesion_parameter_(c_parameter),
      max_speed_(max_speed) {};

void Flock::add_boids(const Boid& new_boid) { boids_.push_back(new_boid); }

void Flock::update_boids(const float& delta_t) {
  for (auto& boid : boids_) {
    boid.change_vel(find_deltav(boid));
    boid.limit_velocity(max_speed_);
    const Vector delta_pos = boid.get_vel() * delta_t;
    boid.change_pos(delta_pos);
  }
}

std::vector<Boid> Flock::get_boids() const { return boids_; };

Vector Flock::find_separation(const Boid& chosen_boid) const {
  Vector null{};
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  assert(near_boid.size() <= boids_.size());
  null = chosen_boid.separation(separation_parameter_, distance_of_separation_,
                                near_boid);

  return null;
}

Vector Flock::find_alignment(const Boid& chosen_boid) const {
  Vector null{};
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  assert(near_boid.size() <= boids_.size());
  null = chosen_boid.alignment(allignment_parameter_, near_boid);

  return null;
}

Vector Flock::find_cohesion(const Boid& chosen_boid) const {
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  assert(near_boid.size() <= boids_.size());
  Vector null{};
  null = chosen_boid.cohesion(cohesion_parameter_, near_boid);

  return null;
}

Vector Flock::find_deltav(const Boid& chosen_boid) const {
  const Vector delta_velocity = find_separation(chosen_boid) +
                                find_alignment(chosen_boid) +
                                find_cohesion(chosen_boid);

  return delta_velocity;
};

Statistics Flock::state() const {
  if (boids_.size() > 2) {
    size_t n = boids_.size();
    size_t num_pairs = (n * (n - 1)) / 2;

    float sum_dist = 0.0f;
    for (size_t i = 0; i < boids_.size(); ++i) {
      for (size_t j = i + 1; j < boids_.size(); ++j) {
        sum_dist += boids_[i].get_pos().distance(boids_[j].get_pos());
      }
    }

<<<<<<< HEAD
    float sum_dist2 = 0.0f;
    for(size_t i = 0; i < boids_.size(); ++i) {
      for(size_t j = i + 1; j < boids_.size(); ++j) {
        sum_dist2 += std::pow(boids_[i].get_pos().distance(boids_[j].get_pos()), 2);
=======
    const float medium_dist = sum_dist / num_pairs;

    float sum_dist2 = 0.0f;

    for (size_t i = 0; i < boids_.size(); ++i) {
      for (size_t j = i + 1; j < boids_.size(); ++j) {
        sum_dist2 +=
            std::pow(boids_[i].get_pos().distance(boids_[j].get_pos()), 2);
>>>>>>> fd95a0d1006e6a0880359df6615177e2354838cb
      }
    }
    const float medium_dist_2 = sum_dist2 / num_pairs;

    const float dev_dist = std::sqrt(medium_dist_2 - std::pow(medium_dist, 2));

    const float sum_vel = std::accumulate(
        boids_.begin(), boids_.end(), 0.,
        [](float res, Boid const& b) { return b.get_vel().norm_vector(); });

    const float medium_speed = sum_vel / boids_.size();

    const float sum_vel2 = std::accumulate(
        boids_.begin(), boids_.end(), 0., [](float res, Boid const& b) {
          return std::pow(b.get_vel().norm_vector(), 2);
        });
    const float medium_speed_2 = sum_vel2 / boids_.size();

    const float dev_speed =
        std::sqrt(medium_speed_2 - std::pow(medium_speed, 2));

    return {medium_dist, dev_dist, medium_speed, dev_speed};
  } else {
    return {0., 0., 0., 0.};
  }
}
}  // namespace sim
