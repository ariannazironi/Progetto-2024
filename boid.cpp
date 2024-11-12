#include "boid.hpp"
#include <vector>

namespace sim {

Boid::Boid(Vector position, Vector velocity)
    : position_(position), velocity_(velocity) {};

Vector Boid::get_pos() const { return position_; };
Vector Boid::get_vel() const { return velocity_; };

//Regola di separazione
Vector Boid::separation(const std::vector<Boid>& boids){

    for(int i = 0; i < boids.size(); ++i ) {
        //calcolo distanza da boid corrente a ciascuno
    }
};

}  // namespace pr