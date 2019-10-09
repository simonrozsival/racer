#include "racer_ros/Follower.h"

#include "racer_ros/utils.h"

namespace racer_ros {

  bool Follower::is_initialized() const {
    return map_.is_valid() && reference_trajectory_.is_valid() && state_.is_valid();
  }

  void Follower::map_observed(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    map_ = msg_to_grid(*map);
    map_frame_id = map->header.frame_id;
  }

  void Follower::trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory) {
    reference_trajectory_ = msg_to_trajectory(*trajectory);
  }

  void Follower::waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints) {
    next_waypoint_ = waypoints->next_waypoint;
  }
      
  void Follower::state_observed(const racer_msgs::State::ConstPtr& state) {
    racer::vehicle_configuration position = { state->x, state->y, state->heading_angle };
    state_ = { position, state->speed, state->steering_angle };
  }

  racer::vehicle_model::kinematic_bicycle_model::action Follower::select_driving_command() const {
    if (reference_trajectory_.is_valid()) {
      return strategy_->select_action(state_, next_waypoint_, reference_trajectory_, map_);
    } else {
      return stop();
    }
  }

  racer::vehicle_model::kinematic_bicycle_model::action Follower::stop() const {
    bool is_moving = std::abs(state_.speed()) > 0.1;
    if (is_moving) {
      double braking_direction = state_.speed() < 0 ? 1.0 : -1.0;
      return { braking_direction, 0.0 };
    } else {
      return stop_;
    }
  }

}