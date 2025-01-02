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

  std::vector<Boid> find_near(const std::vector<Boid>&, float distance) const;
  Vector separation(float s_parameter, float ds_parameter,
                    std::vector<Boid> const&) const;
  Vector alignment(float a_parameter, const std::vector<Boid>&) const;
  Vector cohesion(float c_parameter, const std::vector<Boid>&) const;
  void limit_velocity(float max_speed);
  void min_velocity(float min_speed);
  void change_vel(const Vector& delta_velocity);
  void change_pos(const Vector& delta_position);
  float diff_angle(const Boid& other) const;
  float get_rotation_angle() const;
  bool operator==(const Boid& other_boid) const;
  void border(float x_max, float y_max);
  void set_position(const Vector& new_pos);
  sf::CircleShape set_shape(bool is_predator);
};

}  // namespace sim

#endif