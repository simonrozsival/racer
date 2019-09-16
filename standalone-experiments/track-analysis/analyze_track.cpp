#include <iostream>
#include <filesystem>
#include <fstream>

#include "track.h"

#include "racer/math/primitives.h"
#include "racer/track_analysis.h"

int main(int argc, char *argv[])
{
  std::cout << "Track analysis standalone experiment - The Racer project" << std::endl;

  if (argc != 2)
  {
    std::cerr << "Exactly one argument is requried - path of a circuit definition file." << std::endl;
    return 1;
  }

  std::filesystem::path input_file_name = argv[1];
  std::ifstream file(input_file_name, std::ifstream::in);

  if (!file.is_open())
  {
    std::cerr << "Cannot open circuit definition file '" << std::filesystem::absolute(input_file_name) << "'." << std::endl;
    return 2;
  }
  else
  {
    std::cout << "Loading configuration from file '" << std::filesystem::absolute(input_file_name) << "'." << std::endl;
  }

  // We expect the circuit definition to be in the standard input.
  // The circuit definition has three parts:
  // - the occupancy grid as a PGM file (first line of STDIN) + resolution (second line of STDIN)
  // - space exploration params: radius of the vehicle (third line)
  // - the list of endpoints (two doubles per line - until the end of STDIN)
  std::string pgm_file_name;
  double occupancy_grid_resolution;

  getline(file, pgm_file_name);
  pgm_file_name.erase(remove_if(pgm_file_name.begin(), pgm_file_name.end(), isspace), pgm_file_name.end());

  std::stringstream ss;
  ss << file.rdbuf();

  ss >> occupancy_grid_resolution;

  std::filesystem::path pgm_file_path = pgm_file_name;
  std::cout << "Loading occupancy grid from file '" << pgm_file_name << "' with the resolution of " << occupancy_grid_resolution << "m." << std::endl;

  std::unique_ptr<racer::occupancy_grid> occupancy_grid = load_occupancy_grid(pgm_file_name, occupancy_grid_resolution);
  if (!occupancy_grid)
  {
    std::cerr << "Cannot load occupancy grid from file '" << pgm_file_name << "'." << std::endl;
    return 3;
  }

  // load space exploration params
  double radius, min_distance_between_waypoints, max_distance_between_waypoints;
  int neighbor_circles;
  ss >> radius;
  ss >> neighbor_circles;
  ss >> min_distance_between_waypoints;
  ss >> max_distance_between_waypoints;

  // load initial position
  double ix, iy, itheta;
  ss >> ix;
  ss >> iy;
  ss >> itheta;
  racer::vehicle_position initial_position(ix, iy, itheta);

  // load checkpoints
  double x, y;
  std::list<racer::math::point> checkpoints;
  while (ss >> x)
  {
    ss >> y;
    checkpoints.emplace_back(x, y);
  }

  checkpoints.emplace_back(initial_position.x, initial_position.y);

  // Step 1: Run just space exploration
  std::cout << "RUN space exploration" << std::endl;
  racer::sehs::space_exploration se(*occupancy_grid, radius, 2 * radius, neighbor_circles);
  const auto circles = se.explore_grid(initial_position, checkpoints);

  std::cout << std::endl;

  std::cout << "x,y,r" << std::endl;
  for (const auto &circle : circles)
  {
    std::cout << circle.center.x << "," << circle.center.y << "," << circle.radius << std::endl;
  }

  std::cout << std::endl;

  // Step 2: Run the full track analysis
  // min_distance_between_waypoints
  std::cout << "RUN track analysis" << std::endl;
  racer::track_analysis analysis(*occupancy_grid, max_distance_between_waypoints, neighbor_circles);
  const auto waypoints = analysis.find_apexes(radius, initial_position, checkpoints);

  std::cout << std::endl;

  std::cout << "x,y" << std::endl;
  for (const auto &waypoint : waypoints)
  {
    std::cout << waypoint.x << "," << waypoint.y << std::endl;
  }

  std::cout << std::endl;

  std::cout << "Done." << std::endl;
  return 0;
}