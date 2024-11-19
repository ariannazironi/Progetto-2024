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
void Flock::update_boids(const float& delta_t) {
  for (auto& boid : boids_) {
    auto near = boid.find_near(boids_, closeness_parameter_);
    Vector v = boid.get_vel();
    Vector v1 =
        boid.separation(separation_parameter_, distance_of_separation_, near);
    Vector v2 = boid.cohesion(closeness_parameter_, near);
    Vector v3 = boid.alignment(allignment_parameter_, near);
    Vector new_vel = v + v1 + v2 + v3;
    boid.set_vel(new_vel);
    boid.limit_velocity(max_speed_);
    Vector new_pos = boid.get_pos() + (new_vel * delta_t);
    boid.set_pos(new_pos);
  }
};
std::vector<Boid> Flock::get_boids() const { return boids_; };

Vector Flock::find_centermass(const Boid& chosen_boid) const {
  std::vector<Vector> pos_boid;

  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  for (const auto& boid : near_boid) {
    pos_boid.push_back(boid.get_pos());
  }
  Vector x_sum = std::accumulate(pos_boid.begin(), pos_boid.end(),
                                 Vector{0.f, 0.f}, [](Vector a, Vector b) {
                                   Vector result = a + b;

                                   return result;
                                 });
  Vector mass_center = x_sum * (1.0f / pos_boid.size());
  return mass_center;
}

Vector Flock::find_separation(const Boid& chosen_boid) const {
  Vector null{};
  auto near_boid = chosen_boid.find_near(boids_, closeness_parameter_);
  null = chosen_boid.separation(separation_parameter_, distance_of_separation_,
                                 near_boid);

  return null;
}
}  // namespace sim

// namespace sim
