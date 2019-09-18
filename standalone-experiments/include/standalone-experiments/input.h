#ifndef INPUT_H_
#define INPUT_H_

#include <iostream>
#include <fstream>
#include <filesystem>

#include "parse_pgm.h"

class track_analysis_input
{
public:
  const racer::vehicle_position initial_position;
  const std::list<racer::math::point> checkpoints;
  const std::unique_ptr<racer::occupancy_grid> occupancy_grid;
  const double radius;
  const int neighbor_circles;
  const double min_distance_between_waypoints;
  const double max_distance_between_waypoints;

public:
  static std::unique_ptr<track_analysis_input> load(std::filesystem::path input_file_name)
  {
    std::ifstream file(input_file_name, std::ifstream::in);
    if (!file.is_open())
    {
      std::cerr << "Cannot open circuit definition file '" << std::filesystem::absolute(input_file_name) << "'." << std::endl;
      return nullptr;
    }
    else
    {
      std::cout << "Loading configuration from file '" << std::filesystem::absolute(input_file_name) << "'." << std::endl;
    }

    // load occupancy grid
    std::string pgm_file_name;
    double occupancy_grid_resolution;

    getline(file, pgm_file_name);
    pgm_file_name.erase(remove_if(pgm_file_name.begin(), pgm_file_name.end(), isspace), pgm_file_name.end());

    std::filesystem::path pgm_file_path = input_file_name.parent_path() / pgm_file_name;

    std::stringstream ss;
    ss << file.rdbuf();

    ss >> occupancy_grid_resolution;

    std::cout << "Loading occupancy grid from file '" << pgm_file_path << "' with the resolution of " << occupancy_grid_resolution << "m." << std::endl;
    auto occupancy_grid = load_occupancy_grid_from_pgm(pgm_file_path, occupancy_grid_resolution);
    if (!occupancy_grid)
    {
      std::cerr << "Cannot load occupancy grid from file '" << pgm_file_path << "'." << std::endl;
      return nullptr;
    }

    std::cout << "Load config: " << std::endl;

    // load space exploration params
    double radius, min_distance_between_waypoints, max_distance_between_waypoints;
    int neighbor_circles;
    ss >> radius;
    ss >> neighbor_circles;
    ss >> min_distance_between_waypoints;
    ss >> max_distance_between_waypoints;

    std::cout << "radius: " << radius << std::endl;
    std::cout << "neighbor_circles: " << neighbor_circles << std::endl;
    std::cout << "min_distance_between_waypoints: " << min_distance_between_waypoints << std::endl;
    std::cout << "max_distance_between_waypoints: " << max_distance_between_waypoints << std::endl;

    // load initial position
    double ix, iy, itheta;
    ss >> ix;
    ss >> iy;
    ss >> itheta;
    racer::vehicle_position initial_position(ix * occupancy_grid_resolution, iy * occupancy_grid_resolution, itheta);

    std::cout << "initial position: [" << initial_position.x << ", " << initial_position.y << "], theta: " << initial_position.heading_angle << std::endl;

    // load checkpoints
    double x, y;
    std::list<racer::math::point> checkpoints;
    while (ss >> x)
    {
      ss >> y;
      checkpoints.emplace_back(x * occupancy_grid_resolution, y * occupancy_grid_resolution);
      std::cout << "checkpoint: [" << checkpoints.back().x << ", " << checkpoints.back().y << "]" << std::endl;
    }

    checkpoints.emplace_back(initial_position.x, initial_position.y);

    std::cout << "Config loaded." << std::endl
              << std::endl;

    // put it all together
    return std::make_unique<track_analysis_input>(
        initial_position,
        checkpoints,
        std::move(occupancy_grid),
        radius,
        neighbor_circles,
        min_distance_between_waypoints,
        max_distance_between_waypoints);
  }

  track_analysis_input(
      racer::vehicle_position initial_position,
      std::list<racer::math::point> checkpoints,
      std::unique_ptr<racer::occupancy_grid> occupancy_grid,
      double radius,
      int neighbor_circles,
      double min_distance_between_waypoints,
      double max_distance_between_waypoints) : initial_position(initial_position),
                                               checkpoints(checkpoints),
                                               occupancy_grid(std::move(occupancy_grid)),
                                               radius(radius),
                                               neighbor_circles(neighbor_circles),
                                               min_distance_between_waypoints(min_distance_between_waypoints),
                                               max_distance_between_waypoints(max_distance_between_waypoints)
  {
  }
};

#endif