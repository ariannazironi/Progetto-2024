#include "flock.hpp"

#include <fstream>
#include <iostream>

int main() {
  std::cout << "Boid Simulation: 5 Boids starting at the edges, moving toward "
               "the center.\n";

  sim::Flock flock(100.0f, 30.0f, 0.05f, 0.5f, 0.0005f, 100.0f, 10.0f);

  sim::Vector pos1(0.0f, 0.0f);
  sim::Vector pos2(800.0f, 0.0f);
  sim::Vector pos3(0.0f, 600.0f);
  sim::Vector pos4(800.0f, 600.0f);
  sim::Vector pos5(0.0f, 300.0f);

  sim::Vector vel1(12.0f, 15.0f);
  sim::Vector vel2(-20.0f, 5.0f);
  sim::Vector vel3(15.0f, -30.0f);
  sim::Vector vel4(-20.0f, -5.0f);
  sim::Vector vel5(10.0f, 0.0f);

  sim::Boid boid_1(pos1, vel1, 180.0f);
  sim::Boid boid_2(pos2, vel2, 180.0f);
  sim::Boid boid_3(pos3, vel3, 180.0f);
  sim::Boid boid_4(pos4, vel4, 180.0f);
  sim::Boid boid_5(pos5, vel5, 180.0f);

  flock.add_boids(boid_1);
  flock.add_boids(boid_2);
  flock.add_boids(boid_3);
  flock.add_boids(boid_4);
  flock.add_boids(boid_5);

  std::ofstream output_file("simulation_data.txt");
  if (!output_file.is_open()) {
    std::cerr << "Errore nell'aprire il file di output.\n";
    return 1;
  }

  output_file << "Time Mean Distance Standard Deviation of Distance Mean "
                 "Speed Standard Deviation of Speed\n";

  float time_passed = 0.0f;
  float simulation_time = 35.0f;  

  while (time_passed < simulation_time) {
    float delta_t = 0.5f;
    time_passed += delta_t;

    flock.update_boids(delta_t, 800, 600);

    if (time_passed >= 1.0f) {
      const sim::Statistics flock_state = flock.state();
      output_file << time_passed << " " << flock_state.mean_dist << " "
                  << flock_state.dev_dist << " " << flock_state.mean_speed
                  << " " << flock_state.dev_speed << "\n";
    }
  }

  output_file.close();
  std::cout << "Dati salvati in simulation_data.txt.\n";
  return 0;
}
