#include <chrono>
#include <iostream>

#include "standalone-experiments/input.h"
#include "standalone-experiments/plot.h"

#include "racer/math.h"
#include "racer/space_exploration/space_exploration.h"

#include "racer/track/centerline.h"
#include "racer/track/analysis.h"

void stop_stopwatch(
    std::string name,
    const std::chrono::time_point<std::chrono::steady_clock> &start)
{
  const auto end = std::chrono::steady_clock::now();
  std::cout << name << " took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
}

int main(int argc, char *argv[])
{
  std::cout << "Track analysis standalone experiment - The Racer project"
            << std::endl
            << std::endl;

  if (argc != 2)
  {
    std::cerr << "Exactly one argument is requried - path of a circuit "
                 "definition file."
              << std::endl;
    return 1;
  }

  std::filesystem::path input_file_name = argv[1];

  const auto config = track_analysis_input::load(input_file_name);
  if (!config)
  {
    std::cerr << "Loading configuration from '" << input_file_name
              << "' failed." << std::endl;
    return 2;
  }

  // Step 0
  std::vector<racer::math::point> final_check_points{
      config->checkpoints.begin(), config->checkpoints.end()};
  final_check_points.push_back(
      config->initial_position.location()); // back to the start

  std::cout << "Find centerline" << std::endl;
  const auto se_start = std::chrono::steady_clock::now();

  // Step 1
  const auto centerline = racer::track::centerline::find(
      config->initial_position, config->occupancy_grid, final_check_points);
  stop_stopwatch("centerline", se_start);

  if (centerline.empty())
  {
    std::cout << "Finding centerline failed, cannot proceed to track analysis."
              << std::endl;
    return 3;
  }
  else
  {
    std::cout << "Number of centerline points: " << centerline.points().size() << std::endl;
  }
  

  // Step 2
  std::cout << "RUN find pivot points" << std::endl;
  const auto find_pivot_points_start = std::chrono::steady_clock::now();
  racer::track::analysis analysis{centerline.width()};
  const auto pivot_points =
      analysis.find_pivot_points(centerline.points(), config->occupancy_grid);
  stop_stopwatch("find pivot points", find_pivot_points_start);

  std::cout << "Number of pivot points: " << pivot_points.size() << std::endl;

  // Step 3
  std::cout << "RUN find corners" << std::endl;
  const auto find_corners_start = std::chrono::steady_clock::now();
  const auto sharp_turns =
      analysis.remove_insignificant_turns(pivot_points);
  const auto corners = analysis.merge_close(sharp_turns);
  stop_stopwatch("find corners", find_corners_start);

  std::cout << "Number of detected corners: " << corners.size() << std::endl;

  if (!corners.empty())
  {
    // This requires Linux or WSL+Xserver
    std::cout << "Show interactive plot" << std::endl;
    plot_track_analysis(*config, centerline, pivot_points, corners, centerline.width());
  }
  else
  {
    std::cout << "Could not detect any corners. Is the map empty?" << std::endl;
  }

  std::cout << "Done." << std::endl;
  return 0;

}