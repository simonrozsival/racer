#include "Follower.h"

#include <iostream>
#include <cstdlib>
#include "utils.h"

bool Follower::is_initialized() const {
  return costmap_ != nullptr && reference_trajectory_ != nullptr && last_imu_message_time_ > 0;
}

void Follower::costmap_observed(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  costmap_ = std::move(msg_to_grid(*map));
  map_frame_id = map->header.frame_id;
}

void Follower::trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory) {
  reference_trajectory_ = std::move(msg_to_trajectory(*trajectory));
}

void Follower::waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints) {
  next_waypoint_ = waypoints->next_waypoint;
}
    
void Follower::imu_observed(const sensor_msgs::Imu::ConstPtr& imu) {
  if (last_imu_message_time_ > 0) {  
    double acc = imu->linear_acceleration.x;
    double dt = imu->header.stamp.sec - last_imu_message_time_;

    current_speed_ = acc * dt; // euler integration
  }

  last_imu_message_time_ = imu->header.stamp.sec;
}

const racing::kinematic_model::state Follower::last_known_state() const {
  tf::StampedTransform transform;
  tf_listener_.waitForTransform(map_frame_id, base_link_frame_id_, ros::Time(0), ros::Duration(0.1));
  tf_listener_.lookupTransform(map_frame_id, base_link_frame_id_, ros::Time(0), transform);

  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  racing::vehicle_position position(origin.x(), origin.y(), rotation);
  double steering_angle = 0; // we assume the steering angle is 0
  return racing::kinematic_model::state(position, current_speed_, steering_angle);
}

std::unique_ptr<racing::kinematic_model::action> Follower::select_driving_command() const {
  const auto state = last_known_state();
  if (reference_trajectory_ && reference_trajectory_->steps.size() > 0) {
    return strategy_->select_action(state, next_waypoint_, *reference_trajectory_, *costmap_);
  } else {
    return stop();
  }
}

std::unique_ptr<racing::kinematic_model::action> Follower::stop() const {
  const auto state = last_known_state();
  bool is_moving = std::abs(state.speed) > 0.1;
  if (is_moving) {
    double braking_direction = state.speed < 0 ? 1.0 : -1.0;
    return std::make_unique<racing::kinematic_model::action>(braking_direction, 0.0);
  } else {
    return std::make_unique<racing::kinematic_model::action>(stop_);
  }
}
