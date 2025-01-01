#include "flock.hpp"

#include <cassert>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace sim {

Flock::Flock(const float distance, const float ds_parameter,
             const float s_parameter, const float a_parameter,
             const float c_parameter, const float max_speed,
             const float min_speed)
    : closeness_parameter_(distance),
      distance_of_separation_(ds_parameter),
      separation_parameter_(s_parameter),
      allignment_parameter_(a_parameter),
      cohesion_parameter_(c_parameter),
      max_speed_(max_speed),
      min_speed_(min_speed) {};

void Flock::add_boids(const Boid& new_boid) { boids_.push_back(new_boid); }
void Flock::add_predators(const Boid& new_predator) {
  predators_.push_back(new_predator);
}

std::vector<Boid> Flock::get_boids() const { return boids_; };
std::vector<Boid> Flock::get_predators() const { return predators_; };

Vector Flock::find_separation(const Boid& chosen_boid) const {
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  assert(near_boid.size() <= boids_.size());
  
  Vector null{};
  null = chosen_boid.separation(separation_parameter_, distance_of_separation_,
                                near_boid);
  for (auto& predator : predators_) {
    float predator_dist = chosen_boid.get_pos().distance(predator.get_pos());
    if (predator_dist < distance_of_separation_) {
      null += (predator.get_pos() - chosen_boid.get_pos()) *
              (-separation_parameter_);
    }
  }
  return null;
}

Vector Flock::find_alignment(const Boid& chosen_boid) const {
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  assert(near_boid.size() <= boids_.size());

  Vector null{};
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

void Flock::update_entity(Boid& entity, const Vector delta_v,
                          const float& delta_t, const float x_max,
                          const float y_max) {
  entity.change_vel(delta_v);
  entity.limit_velocity(max_speed_);
  entity.min_velocity(min_speed_);
  const Vector delta_pos = entity.get_vel() * delta_t;
  entity.change_pos(delta_pos);
  entity.border(x_max, y_max);
}

void Flock::update_boids(const float& delta_t, const float x_max,
                         const float y_max) {
  for (auto& boid : boids_) {
    Vector escape_vel{};
    for (const auto& predator : predators_) {
      float distance = boid.get_pos().distance(predator.get_pos());
      if (distance < 150.0f) {
        escape_vel += (boid.get_pos() - predator.get_pos());
      }
    }
    Vector delta_v = escape_vel + find_deltav(boid);
    update_entity(boid, delta_v, delta_t, x_max, y_max);
  }
}

Boid Flock::find_prey(const Boid& predator) const {
  assert(!boids_.empty());
  auto prey = *std::min_element(boids_.begin(), boids_.end(),
                           [&](const Boid& a, const Boid& b) {
                             return predator.get_pos().distance(a.get_pos()) <
                                    predator.get_pos().distance(b.get_pos());
                           });
  return prey;
}

void Flock::update_predator(const float& delta_t, const float x_max,
                            const float y_max) {
  for (auto& predator : predators_) {
    Boid prey = find_prey(predator);
    Vector chase_vel =
        (prey.get_pos() - predator.get_pos()) * separation_parameter_;

    auto near_predator = predator.find_near(predators_, closeness_parameter_);
    assert(near_predator.size() <= predators_.size());
    
    Vector separation_vel = predator.separation(
        separation_parameter_, distance_of_separation_, near_predator);
    Vector delta_v = chase_vel + separation_vel;
    update_entity(predator, delta_v, delta_t, x_max, y_max);
  }
}

Statistics Flock::state() const {
  size_t n = boids_.size();
  if (n >= 2) {
    size_t num_pairs = (n * (n - 1)) / 2;

    float sum_dist = 0.0f;
    float sum_dist2 = 0.0f;

    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i + 1; j < n; ++j) {
        float dist= boids_[i].get_pos().distance(boids_[j].get_pos());
        sum_dist += dist;
        sum_dist2 += std::pow(dist, 2);
      }
    }

    const float medium_dist = sum_dist / num_pairs;
     
    const float medium_dist_2 = sum_dist2 / num_pairs;

    const float dev_dist = std::sqrt(medium_dist_2 - std::pow(medium_dist, 2));

    const float sum_vel = std::accumulate(
        boids_.begin(), boids_.end(), 0., [](float res, Boid const& b) {
          return res + b.get_vel().norm_vector();
        });

    const float medium_speed = sum_vel / n;

    const float sum_vel2 = std::accumulate(
        boids_.begin(), boids_.end(), 0., [](float res, Boid const& b) {
          return res + std::pow(b.get_vel().norm_vector(), 2);
        });
    const float medium_speed_2 = sum_vel2 / n;

    const float dev_speed =
        std::sqrt(medium_speed_2 - std::pow(medium_speed, 2));

    return {medium_dist, dev_dist, medium_speed, dev_speed};
  } else {
    return {0., 0., 0., 0.};
  }
}

}  