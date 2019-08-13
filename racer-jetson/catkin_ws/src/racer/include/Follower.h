#ifndef AGENT_H_
#define AGENT_H_

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/OccupancyGrid.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racing/following_strategies/following_strategy.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"

class Follower {
  public:
    std::string map_frame_id;

    Follower(
      std::unique_ptr<racing::following_strategy> strategy,
      std::string base_link_frame_id)
      : strategy_(std::move(strategy)),
      base_link_frame_id_(base_link_frame_id),
      next_waypoint_(0),
      stop_(racing::kinematic_model::action(0, 0)),
      current_speed_(0),
      last_imu_message_time_(0)
    {
    }

    bool is_initialized() const;

    void imu_observed(const sensor_msgs::Imu::ConstPtr& imu);
    void costmap_observed(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory);
    void waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints);

    const racing::kinematic_model::state last_known_state() const;
    std::unique_ptr<racing::kinematic_model::action> select_driving_command() const;
    std::unique_ptr<racing::kinematic_model::action> stop() const;


    const int next_waypoint() const {  return next_waypoint_; }
    const racing::kinematic_model::trajectory reference_trajectory() const {
      return *reference_trajectory_; // return a copy}
    }

  private:
    const std::unique_ptr<racing::following_strategy> strategy_;
    const racing::kinematic_model::action stop_;
    int next_waypoint_;

    std::string base_link_frame_id_;
    
    double current_speed_;
    double last_imu_message_time_;

    double last_update_;
    std::unique_ptr<racing::occupancy_grid> costmap_;
    std::unique_ptr<racing::kinematic_model::trajectory> reference_trajectory_;
    tf::TransformListener tf_listener_;
};

#endif
