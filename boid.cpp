#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <cassert>
#include <cmath>
#include <numeric>

namespace sim {

Boid::Boid() : position_{Vector{}}, velocity_{Vector{}}, view_angle_(0.f) {};
Boid::Boid(Vector position, Vector velocity, float view_angle)
    : position_(position), velocity_(velocity), view_angle_(view_angle) {}

Vector Boid::get_pos() const { return position_; };
Vector Boid::get_vel() const { return velocity_; };
float Boid::get_angle() const { return view_angle_; };

float Boid::diff_angle(const Boid& other) const {
  Vector direction = other.get_pos() - position_;
  const float dot_product = velocity_.product(direction);
  const float norm_direction = direction.norm_vector();
  const float norm_velocity = velocity_.norm_vector();

  if (norm_velocity == 0.0f || norm_direction == 0.0f) {
    return 0.0f;
  } else {
    float cos_angle = dot_product / (norm_direction * norm_velocity);
    cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);

    const float rad_angle = std::acos(cos_angle);
    const float degree_angle = (rad_angle * 180.f) / M_PI;
    assert(degree_angle <= 360.f && degree_angle >= 0.f);

    return degree_angle;
  }
}

std::vector<Boid> Boid::find_near(const std::vector<Boid>& boids,
                                  float distance) const {
  assert(distance > 0);
  std::vector<Boid> near;
  for (const auto& boid : boids) {
    const float dist = boid.get_pos().distance(position_);
    if (dist > 0 && dist < distance && diff_angle(boid) <= view_angle_) {
      near.push_back(boid);
    }
  }
  return near;
}

Vector Boid::separation(float s_parameter, float ds_parameter,
                        const std::vector<Boid>& near) const {
  if (near.size() != 0) {
    Vector v1{0., 0.};

    for (const auto& boid : near) {
      const Vector x1 = boid.get_pos();
      if (x1.distance(position_) < ds_parameter) {
        v1 += (x1 - position_) * (-s_parameter);
      }
    }
    return v1;
  } else {
    const Vector null{0., 0.};
    return null;
  }
}

Vector Boid::alignment(float a_parameter,
                       const std::vector<Boid>& near) const {
  if (near.size() != 0) {
    const Vector v_sum = std::accumulate(
        near.begin(), near.end(), Vector{0., 0.},
        [](Vector res, Boid const& b) { return res + b.get_vel(); });
    const Vector v2 = (v_sum * (1.0f / near.size()) - velocity_) * a_parameter;
    return v2;
  } else {
    const Vector null = {0., 0.};
    return null;
  }
}

Vector Boid::cohesion(float c_parameter,
                      const std::vector<Boid>& near) const {
  if (near.size() != 0) {
    const Vector x_sum = std::accumulate(
        near.begin(), near.end(), Vector{0., 0.},
        [](Vector res, Boid const& b) { return res + b.get_pos(); });
    const Vector v3 = (x_sum * (1.0f / near.size()) - position_) * c_parameter;
    return v3;
  } else {
    const Vector null{0., 0.};
    return null;
  }
}

void Boid::limit_velocity(float max_speed) {
  if (velocity_.norm_vector() > max_speed) {
    velocity_ = velocity_ * 0.5;
  }
}

void Boid::min_velocity(float min_speed) {
  if (velocity_.norm_vector() < min_speed) {
    velocity_ = velocity_ * 2;
  }
}

void Boid::change_vel(const Vector& delta_velocity) {
  velocity_ += delta_velocity;
}

void Boid::change_pos(const Vector& delta_position) {
  position_ += delta_position;
}

void Boid::border(float x_max, float y_max) {
  if (position_.get_x() <= 0.) {
    position_.set_x(x_max);
  } else if (position_.get_x() >= x_max) {
    position_.set_x(0.);
  }
  if (position_.get_y() <= 0.) {
    position_.set_y(y_max);
  } else if (position_.get_y() >= y_max) {
    position_.set_y(0.);
  }
}

float Boid::get_rotation_angle() const {
  const float angle = atan2(velocity_.get_y(), velocity_.get_x()) * 180.0f / M_PI;
  return angle + 90.0f;
}

bool Boid::operator==(const Boid& other_boid) const {
  return (position_ == other_boid.position_ &&
          velocity_ == other_boid.velocity_ &&
          view_angle_ == other_boid.view_angle_);
}

sf::CircleShape Boid::set_shape(bool is_predator) {
  boidshape_.setPointCount(3);
  boidshape_.setRadius(7.0f);
  boidshape_.setOrigin(7.0f, 7.0f);
  boidshape_.setPosition(position_.get_x(), position_.get_y());
  boidshape_.setRotation(get_rotation_angle());
  boidshape_.setScale(1.f, 1.5f);
  if (is_predator) {
    boidshape_.setFillColor(sf::Color::Red);
  } else {
    boidshape_.setFillColor(sf::Color::Green);
  }
  return boidshape_;
}

void Boid::set_position(const Vector& new_pos) {
  const sf::Vector2f boid_pos{new_pos.get_x(), new_pos.get_y()};
  boidshape_.setPosition(boid_pos);
}

}  // namespace sim