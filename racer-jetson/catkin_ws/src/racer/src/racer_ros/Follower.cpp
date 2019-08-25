#include "racer_ros/Follower.h"

#include "racer_ros/utils.h"

namespace racer_ros {

  bool Follower::is_initialized() const {
    return map_ && reference_trajectory_ && state_;
  }

  void Follower::map_observed(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    map_ = std::move(msg_to_grid(*map));
    map_frame_id = map->header.frame_id;
  }

  void Follower::trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory) {
    reference_trajectory_ = std::move(msg_to_trajectory(*trajectory));
  }

  void Follower::waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints) {
    next_waypoint_ = waypoints->next_waypoint;
  }
      
  void Follower::state_observed(const racer_msgs::State::ConstPtr& state) {
    racer::vehicle_position position(state->x, state->y, state->heading_angle);
    state_ = std::make_unique<racer::vehicle_model::kinematic_bicycle_model::state>(position, state->speed, state->steering_angle);
  }

  std::unique_ptr<racer::vehicle_model::kinematic_bicycle_model::action> Follower::select_driving_command() const {
    if (reference_trajectory_ && reference_trajectory_->steps.size() > 0) {
      return strategy_->select_action(*state_, next_waypoint_, *reference_trajectory_, *map_);
    } else {
      return stop();
    }
  }

  std::unique_ptr<racer::vehicle_model::kinematic_bicycle_model::action> Follower::stop() const {
    bool is_moving = std::abs(state_->speed) > 0.1;
    if (is_moving) {
      double braking_direction = state_->speed < 0 ? 1.0 : -1.0;
      return std::make_unique<racer::vehicle_model::kinematic_bicycle_model::action>(braking_direction, 0.0);
    } else {
      return std::make_unique<racer::vehicle_model::kinematic_bicycle_model::action>(stop_);
    }
  }

}