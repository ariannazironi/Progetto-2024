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
  sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();
  sf::RenderWindow window(
      sf::VideoMode(desktopMode.width - 20, desktopMode.height - 80),
      "Boid Simulation", sf::Style::Default);
  window.setPosition(sf::Vector2i(0, 0));
  sf::Texture skyTexture;
  // window.setFramerateLimit(200);
  sf::Sprite skySprite;

  skySprite.setTexture(skyTexture);

  sf::Vector2u windowSize = window.getSize();
  sf::Vector2u textureSize = skyTexture.getSize();
  skySprite.setScale(static_cast<float>(windowSize.x) / textureSize.x,
                     static_cast<float>(windowSize.y) / textureSize.y);
  sf::Event event;

  sim::Flock flock(100.0f, 30.0f, 0.1f, 0.5f, 0.0001f, 100.0f, 30.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> velocity_distribution(-100.0f, 100.0f);

  sf::Clock clock;
  sf::Clock clock2;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }

      switch (event.type) {
        case sf::Event::MouseButtonPressed: {
          switch (event.mouseButton.button) {
            case sf::Mouse::Left:
              // Se clicchi con il tasto sinistro, aggiungi un boid
              {
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
              break;

            case sf::Mouse::Right:
              // Se clicchi con il tasto destro, aggiungi un predatore
              {
                const sf::Vector2i position = sf::Mouse::getPosition(window);
                const float positionf_x = static_cast<float>(position.x);
                const float positionf_y = static_cast<float>(position.y);
                const sim::Vector position_f{positionf_x, positionf_y};
                const sim::Vector speed{velocity_distribution(gen),
                                        velocity_distribution(gen)};
                sim::Boid predator{position_f, speed, 180.0f};
                predator.set_position(position_f);
                flock.add_predators(predator);
              }
              break;

            default:
              break;
          }
        } break;

        default:
          break;
      }
    }

    float delta_t = clock.restart().asSeconds();

    flock.update_boids(delta_t , windowSize.x, windowSize.y);
    flock.update_predator(delta_t , windowSize.x, windowSize.y);

    sf::Time time_passed = clock2.getElapsedTime();

    const sim::Statistics flock_state = flock.state();

    if (time_passed.asSeconds() >= 2.f) {
      std::cout << "Medium velocity: " << flock_state.mean_speed<< " +/- "
                << flock_state.dev_speed << ";       "
                << "Medium distance among boids: "
                << flock_state.mean_dist << " +/- "
                << flock_state.dev_dist << ";\n";

      clock2.restart();
    }

    window.clear(sf::Color::Blue);  // pulisce la scena

    for (auto& boid : flock.get_boids()) {
      window.draw(boid.set_shape_boid());
    }

    for (auto& predator : flock.get_predators()) {
      window.draw(predator.set_shape_predator());
    }
    window.display();
  }
  return 0;
}