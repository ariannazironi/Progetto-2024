#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <random>
#include <vector>

#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"

int main() {
  sf::Clock clock;
  sf::RenderWindow window(
      sf::VideoMode(600, 600),
      "Boid Simulation");  // apro finestra nera 600 x 600 con titolo dato

  sim::Flock flock(150.0f, 50.0f, 3.0f, 0.4f, 0.3f, 20.0f);

  const float x_min = 0.0f;
  const float x_max = 600.0f;  // Larghezza della finestra
  const float y_min = 0.0f;
  const float y_max = 600.0f;     // Altezza della finestra

  for (int i = 0; i < 15; ++i) {  // Aggiungi 10 boid casuali
    sim::Boid b = flock.generate_random_boid(0, 600, 0, 600, -5, 5, -5, 5);
    flock.add_boids(b);
  };

  sf::CircleShape boid_shape(5);
  boid_shape.setFillColor(sf::Color::White);

  const float time_step = 0.1f;

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    float delta_t = clock.restart().asSeconds();

    flock.update_boids(delta_t, x_min, x_max, y_min, y_max);

    window.clear(sf::Color::Black);  // pulisce la scena

    for (const auto& boid : flock.get_boids()) {
      auto pos = boid.get_pos();
      boid_shape.setPosition(pos.get_x(), pos.get_y());
      window.draw(boid_shape);
    }
    window.display();  // metto su display ciò che disegno
  }
  return 0;
}