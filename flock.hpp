#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"

namespace sim {

class Flock {
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
  void update_boids(const float& delta_t);
  Vector find_centermass(const Boid& chosen_boid) const;
  Vector find_separation(const Boid& chosen_boid) const;
};

}  // namespace sim

#endif