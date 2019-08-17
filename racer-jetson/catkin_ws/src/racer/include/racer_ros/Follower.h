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
#include "racer/vehicle_model/kinematic_bicycle_model.h"

using namespace racer::vehicle_model::kinematic_bicycle_model;

namespace racer_ros {

  class Follower {
    public:
      std::string map_frame_id;

      Follower(std::shared_ptr<racer::following_strategies::following_strategy> strategy)
        : strategy_(strategy),
        next_waypoint_(0),
        stop_(action(0, 0)),
        state_(nullptr)
      {
      }

      bool is_initialized() const;

      void state_observed(const racer_msgs::State::ConstPtr& state);
      void map_observed(const nav_msgs::OccupancyGrid::ConstPtr& map);
      void trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory);
      void waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints);


      std::unique_ptr<action> select_driving_command() const;
      std::unique_ptr<action> stop() const;

      const int next_waypoint() const {  return next_waypoint_; }

      const state last_known_state() const {
        return *state_; // copy
      }

      const trajectory reference_trajectory() const {
        return *reference_trajectory_; // copy
      }

      const racer::occupancy_grid map() const {
        return *map_; // copy
      }

    private:
      const std::shared_ptr<racer::following_strategies::following_strategy> strategy_;
      const action stop_;
      int next_waypoint_;

      std::unique_ptr<racer::occupancy_grid> map_;
      std::unique_ptr<trajectory> reference_trajectory_;
      std::unique_ptr<state> state_;
  };

}

#endif
