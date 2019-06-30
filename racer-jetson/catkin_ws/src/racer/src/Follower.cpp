#include "Follower.h"
#include "utils.h"

bool Follower::is_initialized() const {
  return grid_ != nullptr && reference_trajectory_ != nullptr && last_known_state_ != nullptr;
}

void Follower::map_observed(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  grid_ = std::move(msg_to_grid(*map));
  frame_id = map->header.frame_id;
}

void Follower::trajectory_observed(const racer::TrajectoryMsg::ConstPtr& trajectory) {
  reference_trajectory_ = std::move(msg_to_trajectory(*trajectory));
}

void Follower::state_observed(const nav_msgs::Odometry::ConstPtr& odom) {
  last_known_state_ = std::move(msg_to_state(*odom));
}

void Follower::waypoints_observed(const racer::WaypointsMsg::ConstPtr& waypoints) {
  next_waypoint_ = waypoints->next_waypoint;
}

std::unique_ptr<racing::kinematic_model::action> Follower::select_driving_command() const {
  return strategy_.select_action(*last_known_state_, next_waypoint_, *reference_trajectory_, *grid_);
}

