#pragma once

#include <iostream>
#include <vector>

#include "racer/vehicle/action.h"

namespace racer::vehicle
{

template <typename State>
struct trajectory_step
{
private:
  State state_;
  action previous_action_;
  std::size_t passed_waypoints_;
  double timestamp_;

public:
  trajectory_step() : state_{}, previous_action_{}, passed_waypoints_{ 0 }, timestamp_{ 0 }
  {
  }

  trajectory_step(const State &state, const action &previous_action, const std::size_t passed_waypoints,
                  const double timestamp)
    : state_{ state }
    , previous_action_{ previous_action }
    , passed_waypoints_{ passed_waypoints }
    , timestamp_{ timestamp }
  {
  }

  trajectory_step(const trajectory_step &step) = default;
  trajectory_step &operator=(const trajectory_step &step) = default;

  trajectory_step(trajectory_step &&step) = default;
  trajectory_step &operator=(trajectory_step &&step) = default;

  const State &state() const
  {
    return state_;
  }
  const action &previous_action() const
  {
    return previous_action_;
  }
  const std::size_t &passed_waypoints() const
  {
    return passed_waypoints_;
  }
  const double timestamp() const
  {
    return timestamp_;
  }
};

}  // namespace racer
