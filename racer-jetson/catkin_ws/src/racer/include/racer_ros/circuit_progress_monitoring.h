#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <tuple>
#include <mutex>
#include <sstream>

#include "nav_msgs/OccupancyGrid.h"
#include "racer_msgs/State.h"

#include "racer_ros/config/circuit.h"

#include "racer/vehicle_configuration.h"
#include "racer/occupancy_grid.h"

namespace racer_ros
{

class circuit_progress_monitoring
{
private:
  const config::circuit config_;

  std::mutex state_lock_;
  std::mutex analysis_lock_;

  std::shared_ptr<racer::occupancy_grid> grid_;
  std::string frame_id_;

  std::size_t next_waypoint_;
  std::vector<racer::math::circle> waypoints_;

public:
  circuit_progress_monitoring(const config::circuit& config)
    : config_{config}
  {
  }

  void state_update(const racer_msgs::State::ConstPtr &state)
  {
    if (!grid_) return;

    racer::vehicle_configuration current_configuration{state->x, state->y, state->heading_angle};
  
    if (waypoints_.empty())
    {
      calculate_positions_of_waypoints(current_configuration);
      return;
    }

    std::lock_guard<std::mutex> guard(state_lock_);

    const auto wp = waypoints_[next_waypoint_];
    if (wp.contains(current_configuration.location()))
    {
      next_waypoint_ = (next_waypoint_ + 1) % waypoints_.size();
    }
  }

  void map_update(const nav_msgs::OccupancyGrid::ConstPtr &map)
  {
    if (grid_)
    {
      throw std::runtime_error("There is already an existing map.");
    }

    grid_ = racer_ros::msg_to_grid(*map);
    frame_id_ = map->header.frame_id;
  }

  bool is_initialized() const { return !waypoints_.empty(); }
  std::size_t next_waypoint() const { return next_waypoint_; }
  const std::vector<racer::math::circle>& waypoints() const { return waypoints_; }
  std::string frame_id() const { return frame_id_; }

  std::vector<racer::math::circle> waypoints_ahead()
  {
    if (waypoints_.empty()) return {};

    std::lock_guard<std::mutex> guard(analysis_lock_);

    std::vector<racer::math::circle> waypoints;
    for (std::size_t i = 0; i < config_.lookahead; ++i)
    {
      const auto wp = waypoints[(next_waypoint_ + i) % waypoints.size()];
      waypoints.push_back(wp);
    }

    return waypoints;
  }

private:

  void calculate_positions_of_waypoints(const racer::vehicle_configuration& current_configuration)
  {
    if (!current_configuration.is_valid() || !grid_)
    {
      return;
    }

    racer::sehs::space_exploration space_exploration{config_.vehicle_radius, 10 * config_.vehicle_radius, config_.branching_factor};
    racer::track_analysis analysis{config_.min_distance_between_waypoints};

    std::vector<racer::math::point> final_check_points{config_.check_points.begin(), config_.check_points.end()};
    final_check_points.push_back(current_configuration.location()); // back to the start

    const auto path = space_exploration.explore_grid(grid_, current_configuration, final_check_points);
    if (path.empty())
    {
      ROS_ERROR("cannot find path of circles - there might not be enough room for the careful algorithm to fit the car through");
      ROS_DEBUG("select different checkpoints, decrease the radius of the car, or create a new map");
      return;
    }

    const auto pivot_points = analysis.find_pivot_points(path, final_check_points, grid_);
    if (pivot_points.empty())
    {
      ROS_ERROR("cannot find pivot points");
      ROS_DEBUG("select different checkpoints, decrease the radius of the car, or create a new map");
      return;
    }

    const auto apexes = analysis.find_corners(pivot_points, final_check_points, M_PI * 4.0 / 5.0);
    if (apexes.empty())
    {
      ROS_ERROR("cannot find any corners of the track");
      ROS_DEBUG("select different checkpoints, decrease the radius of the car, or create a new map");
      return;
    }

    std::lock_guard<std::mutex> guard(analysis_lock_);
    std::vector<racer::math::circle> wps;
    for (const auto &apex : apexes)
    {
      wps.emplace_back(apex, config_.waypoint_radius);
    }

    waypoints_ = std::vector<racer::math::circle>(wps);
    next_waypoint_ = 0;

    ROS_DEBUG("Track analysis is completed. Number of discovered waypoints: %lu", waypoints_.size());
    ROS_DEBUG("Next waypoint: %lu", next_waypoint_);
  }
};

}