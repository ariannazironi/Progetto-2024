#include "boid.hpp"

namespace pr {

Boid::Boid(sim::Vector position, sim::Vector velocity)
    : position_(position), velocity_(velocity) {};

sim::Vector Boid::get_pos() const { return position_; };
sim::Vector Boid::get_vel() const { return velocity_; };
}  // namespace pr