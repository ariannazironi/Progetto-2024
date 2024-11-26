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
  const float allignment_parameter_;
  const float cohesion_parameter_;
  const float max_speed_;
  std::vector<Boid> boids_;

 public:
  Flock(const float distance, const float ds_parameter, const float s_parameter,
        const float a_parameter, const float c_parameter,
        const float max_speed);

  void add_boids(const Boid& new_boid);
  std::vector<Boid> get_boids() const;

  void update_boids(const float& delta_t, const float x_min, const float x_max, const float y_min,
                  const float y_max);

  Vector find_separation(const Boid& chosen_boid) const;
  Vector find_alignment(const Boid& chosen_boid) const;
  Vector find_cohesion(const Boid& chosen_boid) const;
  Vector find_deltav(const Boid& chosen_boid) const;

  Boid generate_random_boid(float x_min, float x_max, float y_min,
                                 float y_max, float vx_min, float vx_max,
                                 float vy_min, float vy_max);

  Statistics state() const;

};

} 

#endif