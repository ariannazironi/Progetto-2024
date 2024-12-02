#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <random>
#include <vector>

#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"

int main() {
  sf::RenderWindow window(
      sf::VideoMode(600, 600),
      "Boid Simulation");  // apro finestra nera 600 x 600 con titolo dato
  sf::Event event;

  const float x_max = 600.0f;  // Larghezza della finestra
  const float y_max = 600.0f;  // Altezza della finestra

  const float vx_min = -10.0f;
  const float vy_min = -10.0f;
  const float vx_max = 10.0f;
  const float vy_max = 10.0f;

  sim::Flock flock(100.0f, 15.0f, 1.5f, 1.0f, 0.2f, 20.0f, 5.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> velocity_distribution(-5.0f, 5.0f);

  while (window.isOpen()) {
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event.type == sf::Event::MouseButtonPressed &&
          event.mouseButton.button == sf::Mouse::Left) {
        const sf::Vector2i position = sf::Mouse::getPosition(window);
        const float positionf_x = static_cast<float>(position.x);
        const float positionf_y = static_cast<float>(position.y);
        const sim::Vector position_f{positionf_x, positionf_y};
        const sim::Vector speed{velocity_distribution(gen),
                                velocity_distribution(gen)};
        sim::Boid boid{position_f, speed, 180.0f};
        boid.set_position(position_f);
        flock.add_boids(boid);
      }

      const float delta_t = 0.1f;

      flock.update_boids(delta_t, x_max, y_max);

      window.clear(sf::Color::Black);  // pulisce la scena

      for (auto& boid : flock.get_boids()) {
        window.draw(boid.set_shape());
      }
      window.display();
    }
  }
  return 0;
}
