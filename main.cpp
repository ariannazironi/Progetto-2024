#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <thread>

#include "flock.hpp"

int main() {
  std::cout << "Boid Simulation, instructions:\n";
  std::cout << "1. First, click the left mouse button to add a Boid.\n";
  std::cout << "2. Then, click the right mouse button to add a predator.\n";
  std::cout << "3. Finally, close the window to stop the simulation.\n";

  std::cout
      << "Insert the following parameters: \n"
      << "1) Closeness parameter (values permitted are between [60, 200]) : \n";
  float closeness_parameter;
  std::cin >> closeness_parameter;
  std::cout << "2) Distance of separation (values permitted are [30, 50]): \n";
  float distance_of_separation;
  std::cin >> distance_of_separation;
  std::cout << "3) Separation parameter (values permitted are between [0.3, "
               "0.5]): \n";
  float separation_parameter;
  std::cin >> separation_parameter;
  std::cout << "4) Alignement parameter (values permitted are between [0.4, "
               "0.8]): \n";
  float alignement_parameter;
  std::cin >> alignement_parameter;
  std::cout << "5) Cohesion parameter (values permitted are between [0.0001, "
               "0.0004]): \n";
  float cohesion_parameter;
  std::cin >> cohesion_parameter;

  sf::Clock delay_clock;

  while (delay_clock.getElapsedTime().asSeconds() < 3.0f) {
    std::cout << "The simulation will start in "
              << (3 - (int)delay_clock.getElapsedTime().asSeconds())
              << " seconds...\r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << std::endl;
  std::cout << std::endl;

  sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();
  sf::RenderWindow window(
      sf::VideoMode(desktopMode.width - 20, desktopMode.height - 80),
      "Boid Simulation", sf::Style::Default);
  window.setPosition(sf::Vector2i(0, 0));

  sf::Vector2u windowSize = window.getSize();

  sim::Flock flock(closeness_parameter, distance_of_separation,
                   separation_parameter, alignement_parameter,
                   cohesion_parameter, 100.0f, 30.0f);

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<float> velocity_distribution(-100.0f, 100.0f);
  std::uniform_real_distribution<float> angle_distribution(120.0f, 180.0f);

  sf::Clock clock;
  sf::Clock statistics_clock;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }

      switch (event.type) {
        case sf::Event::MouseButtonPressed: {
          switch (event.mouseButton.button) {
            case sf::Mouse::Left: {
              const sf::Vector2i position = sf::Mouse::getPosition(window);
              const float positionf_x = static_cast<float>(position.x);
              const float positionf_y = static_cast<float>(position.y);
              const sim::Vector position_f{positionf_x, positionf_y};
              const sim::Vector speed{velocity_distribution(gen),
                                      velocity_distribution(gen)};
              const float view_angle = angle_distribution(gen);
              sim::Boid boid{position_f, speed, view_angle};
              boid.set_position(position_f);
              flock.add_boids(boid);
            } break;

            case sf::Mouse::Right: {
              const sf::Vector2i position = sf::Mouse::getPosition(window);
              const float positionf_x = static_cast<float>(position.x);
              const float positionf_y = static_cast<float>(position.y);
              const sim::Vector position_f{positionf_x, positionf_y};
              const sim::Vector speed{velocity_distribution(gen),
                                      velocity_distribution(gen)};
              const float view_angle = angle_distribution(gen);
              sim::Boid predator{position_f, speed, view_angle};
              predator.set_position(position_f);
              flock.add_predators(predator);
            } break;

            default:
              break;
          }
        } break;

        default:
          break;
      }
    }

    float delta_t = clock.restart().asSeconds();

    flock.update_boids(delta_t, windowSize.x, windowSize.y);
    flock.update_predator(delta_t, windowSize.x, windowSize.y);

    sf::Time time_passed = statistics_clock.getElapsedTime();

    const sim::Statistics flock_state = flock.state();

    if (flock.get_boids().size() >= 2 && time_passed.asSeconds() >= 2.f) {
      std::cout << "Medium velocity: " << flock_state.mean_speed << " +/- "
                << flock_state.dev_speed << ";       "
                << "Medium distance among boids: " << flock_state.mean_dist
                << " +/- " << flock_state.dev_dist << ";\n";

      statistics_clock.restart();
    }

    window.clear(sf::Color::Blue);

    for (auto boid : flock.get_boids()) {
      window.draw(boid.set_shape(false));
    }

    for (auto predator : flock.get_predators()) {
      window.draw(predator.set_shape(true));
    }

    window.display();
  }
  return 0;
}