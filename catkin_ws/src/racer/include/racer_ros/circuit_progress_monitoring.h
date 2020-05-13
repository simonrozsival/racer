#pragma once

#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "racer_msgs/State.h"

#include "racer_ros/config/circuit.h"

#include "racer/track/occupancy_grid.h"
#include "racer/track/centerline.h"
#include "racer/track/analysis.h"
#include "racer/vehicle/configuration.h"

#include "racer_ros/utils.h"

namespace racer_ros
{
  class circuit_progress_monitoring
  {
  private:
    const config::circuit config_;

    std::mutex state_lock_;
    std::mutex analysis_lock_;

    std::shared_ptr<racer::track::occupancy_grid> grid_;
    std::string frame_id_;

    std::size_t next_waypoint_;
    std::vector<racer::math::circle> waypoints_;

  public:
    circuit_progress_monitoring(const config::circuit &config)
        : config_{config} {}

    void state_update(const racer_msgs::State::ConstPtr &state)
    {
      if (!grid_)
        return;

      racer::vehicle::configuration current_configuration{state->x, state->y,
                                                         state->heading_angle};

      if (waypoints_.empty())
      {
        calculate_positions_of_waypoints(current_configuration);
        return;
      }

      std::lock_guard<std::mutex> guard(state_lock_);

      const auto wp = waypoints_[next_waypoint_ % waypoints_.size()];
      if (wp.contains(current_configuration.location()))
      {
        ++next_waypoint_;
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
    const std::vector<racer::math::circle> &waypoints() const
    {
      return waypoints_;
    }
    std::string frame_id() const { return frame_id_; }

    std::vector<racer::math::circle> waypoints_ahead()
    {
      if (waypoints_.empty())
        return {};

      std::lock_guard<std::mutex> guard(analysis_lock_);

      std::vector<racer::math::circle> waypoints;
      for (std::size_t i = 0; i < config_.lookahead; ++i)
      {
        const auto wp = waypoints_[(next_waypoint_ + i) % waypoints_.size()];
        waypoints.push_back(wp);
      }

      return waypoints;
    }

  private:
    void calculate_positions_of_waypoints(
        const racer::vehicle::configuration &current_configuration)
    {
      if (!current_configuration.is_valid() || !grid_)
      {
        return;
      }

      // Step 0
      std::vector<racer::math::point> final_check_points{
          config_.check_points.begin(), config_.check_points.end()};
      final_check_points.push_back(
          current_configuration.location()); // back to the start

      // Step 1
      const auto centerline = racer::track::centerline::find(
          current_configuration, grid_, final_check_points);

      if (centerline.empty())
      {
        ROS_ERROR("cannot find center line");
        return;
      }

      // Step 2
      racer::track::analysis analysis{centerline.width()};
      const auto pivot_points =
          analysis.find_pivot_points(centerline.points(), grid_);
      if (pivot_points.empty())
      {
        ROS_ERROR("cannot find pivot points");
        ROS_DEBUG("select different checkpoints, decrease the radius of the car, "
                  "or create a new map");
        return;
      }

      // Step 3
      const auto sharp_turns =
          analysis.remove_insignificant_turns(pivot_points);
      const auto apexes = analysis.merge_close(sharp_turns);
      if (apexes.empty())
      {
        ROS_ERROR("cannot find any corners of the track");
        ROS_DEBUG("select different checkpoints, decrease the radius of the car, "
                  "or create a new map");
        return;
      }

      std::lock_guard<std::mutex> guard(analysis_lock_);
      std::vector<racer::math::circle> wps;
      for (const auto &apex : apexes)
      {
        wps.emplace_back(apex, centerline.width());
      }

      waypoints_ = std::vector<racer::math::circle>(wps);
      next_waypoint_ = 0;

      ROS_DEBUG(
          "Track analysis is completed. Number of discovered waypoints: %lu",
          waypoints_.size());
      ROS_DEBUG("Next waypoint: %lu", next_waypoint_);
    }
  };

} // namespace racer_ros