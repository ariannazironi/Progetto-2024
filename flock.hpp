#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"

namespace sim {

struct Statistics {
  float mean_dist;
  float dev_dist;
  float mean_speed;
  float dev_speed;
};

class Flock {
 private:
  const float closeness_parameter_;
  const float distance_of_separation_;
  const float separation_parameter_;
  const float alignment_parameter_;
  const float cohesion_parameter_;
  const float max_speed_;
  const float min_speed_;
  std::vector<Boid> boids_;
  std::vector<Boid> predators_;

 public:
  Flock(const float distance, const float ds_parameter, const float s_parameter,
        const float a_parameter, const float c_parameter, const float max_speed,
        const float min_speed);

  void add_boids(const Boid& new_boid);
  void add_predators(const Boid& new_predator);

  std::vector<Boid> get_boids() const;
  std::vector<Boid> get_predators() const;

  Vector find_separation(const Boid& chosen_boid) const;
  Vector find_alignment(const Boid& chosen_boid) const;
  Vector find_cohesion(const Boid& chosen_boid) const;

  Vector find_deltav(const Boid& chosen_boid) const;

  void update_entity(Boid& entity, const Vector& delta_v, float delta_t,
                     float x_max, float y_max);

  void update_boids(float delta_t, float x_max, float y_max);

  Boid find_prey(const Boid& predator) const;

  void update_predator(float delta_t, float x_max, float y_max);

  Statistics state() const;
};

}  // namespace sim

#endif