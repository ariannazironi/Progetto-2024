#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

#include "SFML/Graphics.hpp"
#include "vector.hpp"

namespace sim {

class Boid {
 private:
  Vector position_;
  Vector velocity_;
  float view_angle_;
  sf::CircleShape boidshape_;

 public:
  Boid();
  Boid(Vector position, Vector velocity, float view_angle);
  Vector get_pos() const;
  Vector get_vel() const;
  float get_angle() const;
  float diff_angle(const Boid& other) const;
  std::vector<Boid> find_near(const std::vector<Boid>& boids,
                              const float distance) const;
  Vector separation(const float s_parameter, const float ds_parameter,
                    std::vector<Boid> const&) const;
  Vector alignment(const float a_parameter, std::vector<Boid> const&) const;
  Vector cohesion(const float c_parameter, std::vector<Boid> const&) const;
  void limit_velocity(const float max_speed);
  void min_velocity(const float min_speed);
  void change_vel(const Vector& delta_velocity);
  void change_pos(const Vector& delta_position);
  void border(const float x_max, const float y_max);
  float get_rotation_angle() const;
  void set_position(const Vector& new_pos);
  void set_velocity(const Vector& new_vel);
  bool operator==(const Boid& other_boid) const;
  sf::CircleShape& set_shape_boid();
  sf::CircleShape& set_shape_predator();
  
};

}  // namespace sim

#endif