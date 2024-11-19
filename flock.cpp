#include "flock.hpp"

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
      max_speed_(max_speed){};

void Flock::add_boids(const Boid& new_boid) { boids_.push_back(new_boid); }
void Flock::update_boids(const float& delta_t){
    for(auto& boid: boids_){
     auto near = boid.find_near(boids_, closeness_parameter_ );
     Vector v= boid.get_vel();
     Vector v1= boid.separation(separation_parameter_, distance_of_separation_, boids_);
     Vector v2= boid.cohesion(closeness_parameter_, boids_);
     Vector v3= boid.alignment(allignment_parameter_, boids_);
     Vector new_vel= v + v1 + v2 + v3;
     boid.set_vel(new_vel);
     Vector new_pos = boid.get_pos() + (new_vel * delta_t);
     boid.set_pos(new_pos);
    }
    
};

} 
