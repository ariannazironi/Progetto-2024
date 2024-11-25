#include "flock.hpp"

#include <numeric>
#include <cmath>
#include <algorithm>
#include <vector>
#include <cassert>

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

}

Statistics Flock::state() const{
  if (boids_.size() > 2){
    const float sum_vel = std::accumulate(boids_.begin(), boids_.end(), 0., 
                              [](float res, Boid const& b){
                                return res + b.get_vel().norm_vector();
                              });
    const float medium_speed = sum_vel / boids_.size();

    const float sum_vel2= std::accumulate(boids_.begin(), boids_.end(), 0., 
                              [](float res, Boid const& b){
                                return res + std::pow(b.get_vel().norm_vector(),2);
                              });
    const float medium_speed_2 = sum_vel2 / boids_.size();

    const float dev_speed = std::sqrt( medium_speed_2 - std::pow(medium_speed,2));

    float sum_distance = 0.;
    float sum_distance2 = 0.;

  std::for_each(boids_.begin(), boids_.end(), [&](Boid &b) {
    float min_distance = 0.; 
    bool first = true;
     for (const auto &other : boids_) {
        if (&b != &other) {
         const double distance = (other.get_pos() - b.get_pos()).norm_vector();
            if (first) {
                min_distance = distance;
                first = false;
            } else if (distance < min_distance) {
                min_distance = distance;
            }
        }
      }
    sum_distance += min_distance;
    sum_distance2 += std::pow(min_distance, 2);
  });
    const float medium_dist = sum_distance / boids_.size();
    const float medium_dist_2 = sum_distance2 / boids_.size();
    const float dev_dist = std::sqrt(medium_dist_2- pow(medium_dist, 2));

    return {medium_speed, dev_speed, medium_dist, dev_dist};
  }
else {
  return {0., 0., 0., 0.};
}
}
}

