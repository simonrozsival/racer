#pragma once

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>
#include <racer_msgs/State.h>

#include "racer_ros/utils.h"

#include "racer/following_strategies/following_strategy.h"
#include "racer/occupancy_grid.h"
#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/kinematic_model.h"

using namespace racer::vehicle_model;

namespace racer_ros
{

template <typename State>
class Follower
{
public:
  std::string map_frame_id;

  Follower(
    std::shared_ptr<racer::following_strategies::following_strategy<State>> strategy,
    const double time_step_s)
      : strategy_{strategy},
        stop_{0, 0},
        time_step_s_{time_step_s},
        next_waypoint_{0},
        state_{}
  {
  }

  bool is_initialized() const
  {
    return map_ && reference_trajectory_.is_valid() && state_.is_valid();
  }

  void state_observed(const racer_msgs::State::ConstPtr &state)
  {
    racer::vehicle_configuration position = {state->x, state->y, state->heading_angle};
    state_ = {position, state->motor_rpm, state->steering_angle};
  }

  void map_observed(const nav_msgs::OccupancyGrid::ConstPtr &map)
  {
    map_ = msg_to_grid(*map);
    map_frame_id = map->header.frame_id;
  }

  void trajectory_observed(const racer_msgs::Trajectory::ConstPtr &trajectory)
  {
    reference_trajectory_ = msg_to_trajectory(*trajectory, time_step_s_);
  }

  void waypoints_observed(const racer_msgs::Waypoints::ConstPtr &waypoints)
  {
    next_waypoint_ = waypoints->next_waypoint;
  }

  racer::action select_driving_command() const
  {
    if (reference_trajectory_.is_valid())
    {
      return strategy_->select_action(state_, next_waypoint_, reference_trajectory_, map_);
    }
    else
    {
      ROS_INFO("no valid reference trajectory");
      return stop();
    }
  }

  racer::action stop() const
  {
    bool is_moving_forward = state_.motor_rpm() > 250;
    bool is_moving_backward = state_.motor_rpm() < -250;
    if (is_moving_forward)
    {
      return {-1.0, 0.0};
    }
    else if (is_moving_backward)
    {
      return {1.0, 0.0};
    }
    else
    {
      return stop_;
    }
  }

  const int next_waypoint() const { return next_waypoint_; }

  const State& last_known_state() const
  {
    return state_;
  }

  const racer::trajectory<State>& reference_trajectory() const
  {
    return reference_trajectory_;
  }

  const std::shared_ptr<racer::occupancy_grid> map() const
  {
    return map_;
  }

private:
  const std::shared_ptr<racer::following_strategies::following_strategy<State>> strategy_;
  const racer::action stop_;
  const double time_step_s_;

  int next_waypoint_;
  std::shared_ptr<racer::occupancy_grid> map_;
  racer::trajectory<State> reference_trajectory_;
  State state_;
};

} // namespace racer_ros
