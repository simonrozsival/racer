#ifndef AGENT_H_
#define AGENT_H_

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>
#include <racer_msgs/State.h>

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

  bool is_initialized() const;

  void state_observed(const racer_msgs::State::ConstPtr &state);
  void map_observed(const nav_msgs::OccupancyGrid::ConstPtr &map);
  void trajectory_observed(const racer_msgs::Trajectory::ConstPtr &trajectory);
  void waypoints_observed(const racer_msgs::Waypoints::ConstPtr &waypoints);

  racer::action select_driving_command() const;
  racer::action stop() const;

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

#endif
