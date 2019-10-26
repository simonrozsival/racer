#ifndef INPUT_H_
#define INPUT_H_

#include <iostream>
#include <fstream>
#include <filesystem>

#include "parse_pgm.h"

#include "racer/math.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_configuration.h"

class track_analysis_input
{
public:
  const std::string name;
  const racer::vehicle_configuration initial_position;
  const std::vector<racer::math::point> checkpoints;
  const std::shared_ptr<racer::occupancy_grid> occupancy_grid;
  const double radius;
  const int neighbor_circles;
  const double min_distance_between_waypoints;

public:
  track_analysis_input(
      std::string name,
      racer::vehicle_configuration initial_position,
      std::vector<racer::math::point> checkpoints,
      std::shared_ptr<racer::occupancy_grid> occupancy_grid,
      double radius,
      int neighbor_circles,
      double min_distance_between_waypoints) : name(name),
                                               initial_position(initial_position),
                                               checkpoints(checkpoints),
                                               occupancy_grid(occupancy_grid),
                                               radius(radius),
                                               neighbor_circles(neighbor_circles),
                                               min_distance_between_waypoints(min_distance_between_waypoints)
  {
  }

  static std::unique_ptr<track_analysis_input> load(std::filesystem::path input_file_name)
  {
    std::ifstream file(input_file_name, std::ifstream::in);
    if (!file.is_open())
    {
      std::cerr << "Cannot open circuit definition file '" << std::filesystem::absolute(input_file_name) << "'." << std::endl;
      return nullptr;
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

    auto occupancy_grid = load_occupancy_grid_from_pgm(pgm_file_path, occupancy_grid_resolution, false);
    if (!occupancy_grid)
    {
      std::cerr << "Cannot load occupancy grid from file '" << pgm_file_path << "'." << std::endl;
      return nullptr;
    }

    // load space exploration params
    double radius, min_distance_between_waypoints;
    int neighbor_circles;
    ss >> radius;
    ss >> neighbor_circles;
    ss >> min_distance_between_waypoints;

    // load initial position
    double ix, iy, itheta;
    ss >> ix;
    ss >> iy;
    ss >> itheta;
    racer::vehicle_configuration initial_position(ix * occupancy_grid_resolution, iy * occupancy_grid_resolution, itheta);

    // load checkpoints
    double x, y;
    std::vector<racer::math::point> checkpoints;
    while (ss >> x)
    {
      ss >> y;
      checkpoints.emplace_back(x * occupancy_grid_resolution, y * occupancy_grid_resolution);
    }

    checkpoints.emplace_back(initial_position.location().x(), initial_position.location().y());

    // put it all together
    return std::make_unique<track_analysis_input>(
        input_file_name.stem(),
        initial_position,
        checkpoints,
        std::move(occupancy_grid),
        radius,
        neighbor_circles,
        min_distance_between_waypoints);
  }

  static std::optional<std::vector<std::shared_ptr<track_analysis_input>>> load(int number_of_configs, char *config_file_names[])
  {
    std::vector<std::shared_ptr<track_analysis_input>> configs;
    for (int i = 0; i < number_of_configs; ++i)
    {
      auto input_file_name = std::filesystem::path{config_file_names[i]};
      const std::shared_ptr<track_analysis_input> config = load(input_file_name);
      if (!config)
      {
        std::cerr << "Loading configuration from '" << input_file_name << "' failed." << std::endl;
        return {};
      }

      configs.push_back(config);
    }

    return configs;
  }
};

#endif