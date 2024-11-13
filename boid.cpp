#include "boid.hpp"

#include <cmath>
#include <numeric>

namespace sim {

Boid::Boid() : position_{Vector{}}, velocity_{Vector{}} {};
Boid::Boid(Vector position, Vector velocity)
    : position_(position), velocity_(velocity) {};

Vector Boid::get_pos() const { return position_; };
Vector Boid::get_vel() const { return velocity_; };

std::vector<Boid> Boid::find_near(const std::vector<Boid>& boids, const
                                  float d) const {
  std::vector<Boid> near; //mi riempie un vettore boid con tutti quelli near a seconda del metodo find near

  for (const auto& boid : boids) {
    Vector x = boid.get_pos();
    if (x.distance(position_) >0 && x.distance(position_) < d) {
      near.push_back(boid);
    }
  }
  return near;
}

Vector Boid::separation(float s, float ds,
                        std::vector<Boid> const& near) const {
  Vector v1{0., 0.};

  for (auto it = near.begin(); it != near.end(); it++) {
    Vector x1 = it->get_pos();  // vettore posizione di un boid (it è iteratore del vettore near e x1 la sua posizione)
    if (x1.distance(position_) < ds) {
      v1 += (x1 - position_) * (-s);
    }
  }
  return v1;
}

Vector Boid::alignment(const float a, std::vector<Boid> const& near) const {
  Vector v_sum = std::accumulate( //algoritmo che somma vettori, voglio solo somma su velocità
      near.begin(), near.end(), Vector{0., 0.},
      [](Vector res, Boid const& b) { return res + b.get_vel(); }); //lambda function che modifica algoritmo e prende solo la velocità
  Vector v2 = (v_sum * (1 / near.size()) - velocity_) * a;
  return v2;
}

Vector Boid::cohesion(const float c, std::vector<Boid> const& near) const {
    Vector x_sum = std::accumulate( near.begin(), near.end(), Vector{0.,0.},
    [](Vector res, Boid const& b){ return res + b.get_pos();});
   Vector v3= (x_sum * (1/near.size()) - position_) * c;
   return v3;
}

}  