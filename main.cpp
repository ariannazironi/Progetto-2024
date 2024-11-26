#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include "flock.hpp"
#include "boid.hpp"
#include "vector.hpp"

int main() {
  sf::RenderWindow window(
      sf::VideoMode(600, 800),
      "Boid Simulation");  // apro finestra nera 600 x 600 con titolo dato

  sim::Flock flock(80.0f, 20.0f, 0.1f, 0.2f, 0.3f, 5.0f);

  flock.add_boids(
      sim::Boid(sim::Vector(100.0f, 150.0f), sim::Vector(3.5f, 1.0f)));
  flock.add_boids(
      sim::Boid(sim::Vector(200.0f, 300.0f), sim::Vector(-1.0f, -5.0f)));
  flock.add_boids(
      sim::Boid(sim::Vector(400.0f, 500.0f), sim::Vector(-4.0f, -4.0f)));

  sf::CircleShape boid_shape(5);
  boid_shape.setFillColor(sf::Color::White);

  const float delta_t = 0.1f;

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    flock.update_boids(delta_t);

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