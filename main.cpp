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
      "Boid Simulation");  
  sf::Event event;

  const float x_max = 600.0f;  // Larghezza della finestra
  const float y_max = 600.0f;  // Altezza della finestra

  sim::Flock flock(100.0f, 40.0f, 0.8f, 0.5f, 0.001f, 10.0f, 10.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> velocity_distribution(-10.0f, 10.0f);

  while (window.isOpen()) {
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      
      switch (event.type) {
        case sf::Event::MouseButtonPressed:
          {
            
            switch (event.mouseButton.button) {
              case sf::Mouse::Left:
                // Se clicchi con il tasto sinistro, aggiungi un boid
                {
                  const sf::Vector2i position = sf::Mouse::getPosition(window);
            const float positionf_x = static_cast<float>(position.x);
            const float positionf_y = static_cast<float>(position.y);
            const sim::Vector position_f{positionf_x, positionf_y};
            const sim::Vector speed{velocity_distribution(gen), velocity_distribution(gen)};
                  sim::Boid boid{position_f, speed, 180.0f};
                  boid.set_position(position_f);
                  flock.add_boids(boid);
                }
                break;

             case sf::Mouse::Right:
                // Se clicchi con il tasto destro, aggiungi un predatore
                {
                  const sf::Vector2i position = sf::Mouse::getPosition(window);
            const float positionf_x = static_cast<float>(position.x);
            const float positionf_y = static_cast<float>(position.y);
            const sim::Vector position_f{positionf_x, positionf_y};
            const sim::Vector speed{velocity_distribution(gen), velocity_distribution(gen)};
                  sim::Boid predator{position_f, speed, 180.0f};
                  predator.set_position(position_f);
                  flock.add_predators(predator);
                }
                break;

              default:
                break; 
            }
          }
          break;

        default:
          break; 
      }
    }
    const float delta_t = 0.1f;
  
    flock.update_boids(delta_t, x_max, y_max);
    flock.update_predator(delta_t, x_max, y_max);
    
    window.clear(sf::Color::Black);  // pulisce la scena

    for (auto& boid : flock.get_boids()) {
      window.draw(boid.set_shape_boid());
    }

    for( auto& predator : flock.get_predators()) {
    window.draw(predator.set_shape_predator());
    }
    window.display();
  }
  return 0;
}
