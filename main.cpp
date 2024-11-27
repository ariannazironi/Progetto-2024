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

  sim::Flock flock(130.0f, 15.0f, 3.2f, 1.0f, 0.4f, 50.0f);

  const float x_min = 0.0f;
  const float x_max = 600.0f;  // Larghezza della finestra
  const float y_min = 0.0f;
  const float y_max = 600.0f;  // Altezza della finestra

  for (int i = 0; i < 20; ++i) {  // Aggiungi 10 boid casuali
    sim::Boid b = flock.generate_random_boid(0, 600, 0, 600, -20, 20, -20, 20);
    flock.add_boids(b);
  };

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
      auto vel = boid.get_vel();

      sf::ConvexShape triangle;
      triangle.setPointCount(3);  // creo poligono covesso con 3 vertici
      triangle.setPoint(0, sf::Vector2f(0, -7));
      triangle.setPoint(1, sf::Vector2f(-5, 5));
      triangle.setPoint(2, sf::Vector2f(5, 5));
      triangle.setFillColor(sf::Color::Green);

      triangle.setPosition(pos.get_x(), pos.get_y());
      if (vel.norm_vector() > 0) {
        float angle = std::atan2(vel.get_y(), vel.get_x()) * 180.0f / 3.14159f;
        triangle.setRotation(angle + 90.0f);
      }

      window.draw(triangle);
    }
    window.display();  // metto su display ciò che disegno
  }
  return 0;
}