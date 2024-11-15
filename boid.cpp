#include "boid.hpp"

#include <cmath>
#include <iostream>
#include <numeric>

namespace sim {

Boid::Boid() : position_{Vector{}}, velocity_{Vector{}} {};
Boid::Boid(Vector position, Vector velocity)
    : position_(position), velocity_(velocity) {};

Vector Boid::get_pos() const { return position_; };
Vector Boid::get_vel() const { return velocity_; };

std::vector<Boid> Boid::find_near(const std::vector<Boid>& boids, const
                                  float distance) const {
  std::vector<Boid> near; 

  for (const auto& boid : boids) {
    Vector x = boid.get_pos();
    if (x.distance(position_) >0 && x.distance(position_) < distance) {
      near.push_back(boid);
    }
  }
  return near;
}

void Boid::limit_velocity( const float max_speed) {
  if(velocity_.norm_vector() > max_speed) {
    velocity_ = velocity_ * 0.5;
  }
}

void Boid::set_vel(const Vector& new_velocity) {
    velocity_ = new_velocity;
}

Vector Boid::separation(const float s_parameter, const float ds_parameter,
                        std::vector<Boid> const& near) const {
  Vector v1{0., 0.};

  for (auto it = near.begin(); it != near.end(); it++) {
    Vector x1 = it->get_pos();  
    if (x1.distance(position_) < ds_parameter) {
      v1 += (x1 - position_) * (-s_parameter);
    }
  }
  return v1;
}

Vector Boid::alignment(const float a_parameter, std::vector<Boid> const& near) const {
  Vector v_sum = std::accumulate( 
      near.begin(), near.end(), Vector{0., 0.},
      [](Vector res, Boid const& b) { return res + b.get_vel(); }); 
  Vector v2 = (v_sum * (1.0f / near.size()) - velocity_) * a_parameter;
  return v2;
}

Vector Boid::cohesion(const float c_parameter, std::vector<Boid> const& near) const {
    Vector x_sum = std::accumulate( near.begin(), near.end(), Vector{0.,0.},
    [](Vector res, Boid const& b){ return res + b.get_pos();});
   Vector v3= (x_sum * (1.0f /near.size()) - position_) * c_parameter;
   return v3;
}

}  