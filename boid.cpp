#include "boid.hpp"
#include <vector>

namespace pr {

Boid::Boid(sim::Vector position, sim::Vector velocity)
    : position_(position), velocity_(velocity) {};

sim::Vector Boid::get_pos() const { return position_; };
sim::Vector Boid::get_vel() const { return velocity_; };

//Regola di separazione
sim::Vector Boid::separation(const std::vector<Boid>& boids){

    for(int i = 0; i < boids.size(); ++i ) {
        //calcolo distanza da boid corrente a ciascuno
    }
};

}  // namespace pr