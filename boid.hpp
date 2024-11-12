#ifndef BOID_HPP
#define BOID_HPP

#include "vector.hpp"

namespace sim {
class Boid {
 private:
  Vector position_;
  Vector velocity_;

 public:
  Boid(Vector position, Vector velocity);
  Vector get_pos() const;
  Vector get_vel() const;

  // Metodi per le regole di volo
  Vector separation(const std::vector<Boid>& boid);
  Vector alignment();
  Vector cohesion();
};
}  // namespace pr

#endif