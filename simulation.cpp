#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"  // Include Vector, Boid, Flock

int main() {
  // Parametri del Flock
  const float d = 10.0f;          // Closeness parameter
  const float ds = 4.0f;          // Distance of separation
  const float s = 0.1f;           // Separation parameter
  const float a = 0.2f;           // Alignment parameter
  const float c = 0.3f;           // Cohesion parameter
  const float max_speed = 10.0f;  // Max speed
  const float delta_t = 0.5f;     // Time step
  const float t_max = 10.0f;      // Maximum simulation time

  // Creazione del flock
  sim::Flock flock(d, ds, s, a, c, max_speed);

  // Inizializzazione di boid con posizioni e velocità specifiche
  sim::Vector pos0{0.0f, 0.0f};
  sim::Vector vel0{2.0f, 0.0f};

  sim::Vector pos1{3.0f, 4.0f};
  sim::Vector vel1{1.0f, 1.0f};

  sim::Vector pos2{1.0f, 8.0f};
  sim::Vector vel2{5.0f, -6.0f};

  sim::Vector pos3{2.0f, 2.0f};
  sim::Vector vel3{7.0f, 2.5f};

  // Aggiunta dei boid al flock
  flock.add_boids(sim::Boid(pos0, vel0));
  flock.add_boids(sim::Boid(pos1, vel1));
  flock.add_boids(sim::Boid(pos2, vel2));
  flock.add_boids(sim::Boid(pos3, vel3));

  std::cout << "Tempo\tDistanza media\tVelocità media\tDev. std. "
               "distanza\tDev. std. velocità\n";

  for (float t = 0; t < t_max; t += delta_t) {
    flock.update_boids(delta_t);

    const auto& boids = flock.get_boids();

    sim::Statistics statistics = flock.state();

    std::cout << "  Mean Distance: " << statistics.mean_dist << " ± "
              << statistics.dev_dist << '\n';
    std::cout << "  Mean Speed: " << statistics.mean_speed << " ± "
              << statistics.dev_speed << '\n';
    std::cout << "-----------------------------\n";
  }
}
