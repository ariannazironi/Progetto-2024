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
  window.setVerticalSyncEnabled(false);
  sim::Flock flock(130.0f, 20.0f, 3.2f, 1.4f, 0.6f, 20.0f);
  // Limita il frame rate a 60 FPS

  const float x_max = 600.0f;  // Larghezza della finestra
  const float y_max = 600.0f;  // Altezza della finestra

  for (int i = 0; i < 10; ++i) {  // Aggiungi 10 boid casuali
    sim::Boid b = flock.generate_random_boid(0, 600, 0, 600, -5, 5, -5, 5);
    flock.add_boids(b);
  };

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    float delta_t = 0.1f;

    flock.update_boids(delta_t, x_max, y_max);

    window.clear(sf::Color::Black);  // pulisce la scena

    for (const auto& boid : flock.get_boids()) {
      auto pos = boid.get_pos();
      auto vel = boid.get_vel();

      // Definizione delle dimensioni del triangolo (relativo alla posizione del
      // boid)
      const sf::Vector2f point1(0, -7);  // Punta del triangolo
      const sf::Vector2f point2(-5, 5);  // Base sinistra
      const sf::Vector2f point3(5, 5);   // Base destra

      // Calcola l'angolo di rotazione basato sulla velocità
      float angle = 0;
      if (vel.norm_vector() > 0) {
        angle = std::atan2(vel.get_y(), vel.get_x()) * 180.0f / 3.14159f;
      }

      // Posizione centrale del triangolo
      sf::Vector2f center(pos.get_x(), pos.get_y());

      // Calcola i punti ruotati rispetto al centro
      sf::Transform rotation;
      rotation.rotate(angle, center);  // Rotazione attorno al centro

      // Crea i tre lati del triangolo
      sf::VertexArray triangle(sf::Lines, 6);
      triangle[0].position = rotation.transformPoint(center + point1);
      triangle[1].position = rotation.transformPoint(center + point2);

      triangle[2].position = rotation.transformPoint(center + point2);
      triangle[3].position = rotation.transformPoint(center + point3);

      triangle[4].position = rotation.transformPoint(center + point3);
      triangle[5].position = rotation.transformPoint(center + point1);

      triangle[0].color = sf::Color::Green;
      triangle[1].color = sf::Color::Green;
      triangle[2].color = sf::Color::Green;
      triangle[3].color = sf::Color::Green;
      triangle[4].color = sf::Color::Green;
      triangle[5].color = sf::Color::Green;

      window.draw(triangle);
      window.display();
    }
    return 0;
  }
}
