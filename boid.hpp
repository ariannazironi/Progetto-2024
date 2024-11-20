#ifndef BOID_HPP
#define BOID_HPP

#include <vector>
#include "vector.hpp"

namespace sim {

class Boid {
    private:
    Vector position_;
    Vector velocity_;

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
    void change_vel(const Vector& new_velocity);
    void change_pos(const Vector& new_position);

};

}

#endif