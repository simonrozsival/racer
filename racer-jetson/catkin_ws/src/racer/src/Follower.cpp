#include "Follower.h"

#include <iostream>
#include <cstdlib>
#include "utils.h"

bool Follower::is_initialized() const {
  return costmap_ != nullptr && reference_trajectory_ != nullptr && last_known_state_ != nullptr;
}

void Follower::costmap_observed(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::cout << "COSTMAAAAAP" << std::endl;
  costmap_ = std::move(msg_to_grid(*map));
  frame_id = map->header.frame_id;
}

void Follower::trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory) {
  reference_trajectory_ = std::move(msg_to_trajectory(*trajectory));
}

void Follower::state_observed(const nav_msgs::Odometry::ConstPtr& odom) {
  auto state = std::move(msg_to_state(*odom));

  // my odometry does not contain velocity!!
  if (!last_known_state_) {
    last_known_state_ = std::move(state);
    last_update_ = ros::Time().now().toSec();
    return;
  }

  auto distance = (state->position.location() - last_known_state_->position.location()).length();
  auto time = last_update_ - ros::Time().now().toSec();
  auto speed = time == 0 ? distance / time : 0;

  last_known_state_ = std::make_unique<racing::kinematic_model::state>(
    state->position,
    speed,
    state->steering_angle
  );
  last_update_ = ros::Time().now().toSec();
}

void Follower::waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints) {
  next_waypoint_ = waypoints->next_waypoint;
}

std::unique_ptr<racing::kinematic_model::action> Follower::select_driving_command() const {
  if (reference_trajectory_ && reference_trajectory_->steps.size() > 0) {
    return strategy_->select_action(*last_known_state_, next_waypoint_, *reference_trajectory_, *costmap_);
  } else {
    return std::make_unique<racing::kinematic_model::action>(stop_);
  }
}

std::unique_ptr<racing::kinematic_model::action> Follower::stop() const {
  bool is_moving = std::abs(last_known_state_->speed) > 0.2;
  if (is_moving) {
    double braking_direction = last_known_state_->speed < 0 ? 1.0 : -1.0;
    return std::make_unique<racing::kinematic_model::action>(braking_direction, 0.0);
  } else {
    return std::make_unique<racing::kinematic_model::action>(stop_);
  }
}
