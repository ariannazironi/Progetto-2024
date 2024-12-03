#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <cmath>
#include <cassert>
#include <iostream>
#include <numeric>

namespace sim {

Boid::Boid() : position_{Vector{}}, velocity_{Vector{}}, view_angle(){};
Boid::Boid(Vector position, Vector velocity, const float view_angle)
    : position_(position), velocity_(velocity), view_angle(view_angle) {};

Vector Boid::get_pos() const { return position_; };
Vector Boid::get_vel() const { return velocity_; };
float Boid::get_angle() const {return view_angle;};

float Boid::diff_angle(const Boid& other) const {
    sim::Vector direction = other.get_pos() - position_;
    const float dot_product = velocity_.product(direction);
    const float norm_direction = direction.norm_vector();
    const float norm_velocity = velocity_.norm_vector();
    
    if (norm_velocity == 0.0f || norm_direction == 0.0f) {
        return 0.0f;
    }
    else{
    float cos_angle = dot_product / (norm_direction * norm_velocity);
    cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);
    float rad_angle = std::acos(cos_angle);
    const float degree_angle = (rad_angle * 180.f) / M_PI;

    return degree_angle;
    }
}

std::vector<Boid> Boid::find_near(const std::vector<Boid>& boids,
                                  const float distance) const {
  std::vector<Boid> near;

  for (const auto& boid : boids) {
    Vector x = boid.get_pos();
    if (x.distance(position_) > 0 && x.distance(position_) < distance && diff_angle(boid) <= view_angle) {
      near.push_back(boid);
    }
  }
  return near;
}

Vector Boid::separation(const float s_parameter, const float ds_parameter,
                        std::vector<Boid> const& near) const {
  if (near.size() != 0) {
    Vector v1{0., 0.};

    for (auto it = near.begin(); it != near.end(); it++) {
      Vector x1 = it->get_pos();
      if (x1.distance(position_) < ds_parameter) {
        v1 += (x1 - position_) * (-s_parameter);
      }
    }
    return v1;
  } else {
    Vector null{0., 0.};
    return null;
  }
}

Vector Boid::alignment(const float a_parameter,
                       std::vector<Boid> const& near) const {
  if (near.size() != 0) {
    Vector v_sum = std::accumulate(
        near.begin(), near.end(), Vector{0., 0.},
        [](Vector res, Boid const& b) { return res + b.get_vel(); });
    Vector v2 = (v_sum * (1.0f / near.size()) - velocity_) * a_parameter;
    return v2;
  } else {
    Vector null = {0., 0.};
    return null;
  }
}

Vector Boid::cohesion(const float c_parameter,
                      std::vector<Boid> const& near) const {
  if (near.size() != 0) {
    Vector x_sum = std::accumulate(
        near.begin(), near.end(), Vector{0., 0.},
        [](Vector res, Boid const& b) { return res + b.get_pos(); });
    Vector v3 = (x_sum * (1.0f / near.size()) - position_) * c_parameter;
    return v3;
  } else {
    Vector null{0., 0.};
    return null;
  }
}

void Boid::limit_velocity(const float max_speed) {
  if (velocity_.norm_vector() > max_speed) {
    velocity_ = velocity_ * 0.5;
  }
}

void Boid::change_vel(const Vector& delta_velocity) {
  velocity_ += delta_velocity;
}

void Boid::change_pos(const Vector& delta_position) {
  position_ += delta_position;
}

void Boid::border(const float x_max, const float y_max) {
  if (position_.get_x() <= 0.) {
    position_.set_x(0.);
    velocity_.set_x(-2.0f * velocity_.get_x());
  } else if (position_.get_x() >= x_max) {
    position_.set_x(x_max);
    velocity_.set_x(-2.0f * velocity_.get_x());
  }
  if (position_.get_y() <= 0.) {
    position_.set_y(0.);
    velocity_.set_y(-2.0f * velocity_.get_y());
  } else if (position_.get_y() >= y_max) {
    position_.set_y(y_max);
    velocity_.set_y(-2.0f * velocity_.get_y());
  }
}

/*float Boid::get_vel_angle () const {
  if(velocity_.get_x() >= 0.f && velocity_.get_y() >= 0.f) {
    const float angle = std::atan(std::abs(velocity_.get_y()/velocity_.get_x()))
  * (180.f/M_PI)
  } */
};

 // namespace sim