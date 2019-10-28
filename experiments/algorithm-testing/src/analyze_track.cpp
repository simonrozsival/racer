#include <iostream>
#include <chrono>

#include "standalone-experiments/input.h"
#include "standalone-experiments/plot.h"

#include "racer/math.h"
#include "racer/track_analysis.h"
#include "racer/sehs/space_exploration.h"

void stop_stopwatch(std::string name, const std::chrono::time_point<std::chrono::steady_clock> &start)
{
  const auto end = std::chrono::steady_clock::now();
  std::cout << name << " took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
}

int main(int argc, char *argv[])
{
  std::cout << "Track analysis standalone experiment - The Racer project" << std::endl
            << std::endl;

  if (argc != 2)
  {
    std::cerr << "Exactly one argument is requried - path of a circuit definition file." << std::endl;
    return 1;
  }

  std::filesystem::path input_file_name = argv[1];

  const auto config = track_analysis_input::load(input_file_name);
  if (!config)
  {
    std::cerr << "Loading configuration from '" << input_file_name << "' failed." << std::endl;
    return 2;
  }

  // Step 1
  std::cout << "RUN space exploration" << std::endl;
  const auto se_start = std::chrono::steady_clock::now();
  racer::sehs::space_exploration se(config->radius, 4.0 * config->radius, config->neighbor_circles);
  const auto circles = se.explore_grid(config->occupancy_grid, config->initial_position, config->checkpoints);
  stop_stopwatch("space exploration", se_start);

  if (circles.empty())
  {
    std::cout << "Space exploration failed, cannot proceed to track analysis." << std::endl;
    return 3;
  }

  // Step 2
  std::cout << "RUN find pivot points" << std::endl;
  const auto find_pivot_points_start = std::chrono::steady_clock::now();
  racer::track_analysis analysis(config->min_distance_between_waypoints);
  const auto raw_waypoints = analysis.find_pivot_points(circles, config->occupancy_grid);
  stop_stopwatch("find pivot points", find_pivot_points_start);

  // Step 3
  std::cout << "RUN find corners" << std::endl;
  const double max_angle = 4.0 / 5.0 * M_PI;
  const auto find_corners_start = std::chrono::steady_clock::now();
  const auto waypoints = analysis.find_corners(raw_waypoints, max_angle);
  stop_stopwatch("find corners", find_corners_start);

  // This requires Linux or WSL+Xserver
  std::cout << "Show interactive plot" << std::endl;
  plot_track_analysis(*config, circles, raw_waypoints, waypoints);

  std::cout << "Done." << std::endl;
  return 0;
}