#include "racer_ros/Follower.h"

#include "racer_ros/utils.h"

namespace racer_ros
{

template<typename State>
bool Follower<State>::is_initialized() const
{
  return map_ && reference_trajectory_.is_valid() && state_.is_valid();
}

template<typename State>
void Follower<State>::map_observed(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  map_ = msg_to_grid(*map);
  map_frame_id = map->header.frame_id;
}

template<typename State>
void Follower<State>::trajectory_observed(const racer_msgs::Trajectory::ConstPtr &trajectory)
{
  reference_trajectory_ = msg_to_trajectory(*trajectory, time_step_s_);
}

template<typename State>
void Follower<State>::waypoints_observed(const racer_msgs::Waypoints::ConstPtr &waypoints)
{
  next_waypoint_ = waypoints->next_waypoint;
}

template<typename State>
void Follower<State>::state_observed(const racer_msgs::State::ConstPtr &state)
{
  racer::vehicle_configuration position = {state->x, state->y, state->heading_angle};
  state_ = {position, state->speed, state->steering_angle};
}

template<typename State>
racer::action Follower<State>::select_driving_command() const
{
  if (reference_trajectory_.is_valid())
  {
    return strategy_->select_action(state_, next_waypoint_, reference_trajectory_, map_);
  }
  else
  {
    return stop();
  }
}

template<typename State>
racer::action Follower<State>::stop() const
{
  bool is_moving = std::abs(state_.motor_rpm()) > 250;
  if (is_moving)
  {
    return {-1.0, 0.0};
  }
  else
  {
    return stop_;
  }
}

} // namespace racer_ros