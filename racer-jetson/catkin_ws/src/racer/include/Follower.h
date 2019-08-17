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

#include "racing/following_strategies/following_strategy.h"
#include "racing/collision_detection/occupancy_grid.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"

class Follower {
  public:
    std::string map_frame_id;

    Follower(std::shared_ptr<racing::following_strategy> strategy)
      : strategy_(strategy),
      next_waypoint_(0),
      stop_(racing::kinematic_model::action(0, 0)),
      state_(nullptr)
    {
    }

    bool is_initialized() const;

    void state_observed(const racer_msgs::State::ConstPtr& state);
    void map_observed(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory);
    void waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints);


    std::unique_ptr<racing::kinematic_model::action> select_driving_command() const;
    std::unique_ptr<racing::kinematic_model::action> stop() const;

    const int next_waypoint() const {  return next_waypoint_; }

    const racing::kinematic_model::state last_known_state() const {
      return *state_; // copy
    }

    const racing::kinematic_model::trajectory reference_trajectory() const {
      return *reference_trajectory_; // copy
    }

    const racing::occupancy_grid map() const {
      return *map_; // copy
    }

  private:
    const std::shared_ptr<racing::following_strategy> strategy_;
    const racing::kinematic_model::action stop_;
    int next_waypoint_;

    std::unique_ptr<racing::occupancy_grid> map_;
    std::unique_ptr<racing::kinematic_model::trajectory> reference_trajectory_;
    std::unique_ptr<racing::kinematic_model::state> state_;
};

#endif
