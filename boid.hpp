#ifndef BOID_HPP
#define BOID_HPP

#include <vector>
#include "vector.hpp"

namespace sim {

class Boid {
    private:
    Vector position_;
    Vector velocity_;
    float previous_angle_;

    public:
    Boid();
    Boid(Vector position, Vector velocity);
    Vector get_pos() const;
    Vector get_vel() const;
    std::vector<Boid> find_near(const std::vector<Boid>& boids, float distance) const;
    Vector separation(const float s_parameter, const float ds_parameter, std::vector<Boid> const&) const;
    Vector alignment(const float a_parameter, std::vector<Boid> const&) const;
    Vector cohesion(const float c_parameter, std::vector<Boid> const&) const;
    void limit_velocity(const float max_speed);
    void change_vel(const Vector& delta_velocity);
    void change_pos(const Vector& delta_position);
    void border(const float x_min, const float x_max, const float y_min, const float y_max);
    float get_previous_angle() const;
};

}

#endif