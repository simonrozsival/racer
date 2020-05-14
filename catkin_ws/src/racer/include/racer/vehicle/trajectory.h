#pragma once

#include <iostream>
#include <vector>

#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory_step.h"

namespace racer::vehicle
{

template <typename State>
struct trajectory
{
private:
  std::vector<trajectory_step<State>> steps_;
  double time_step_s_;
  double distance_;

public:
  trajectory() : steps_{}
  {
  }

  trajectory(const std::vector<trajectory_step<State>> steps, const double time_step_s)
    : steps_(steps), time_step_s_{ time_step_s }, distance_{ 0 }
  {
    auto previous_position = steps_.front().state().position();
    for (const auto &step : steps_)
    {
      distance_ += step.state().position().distance(previous_position);
      previous_position = step.state().position();
    }
  }

  trajectory(const trajectory &traj) = default;
  trajectory &operator=(const trajectory &traj) = default;

  trajectory(trajectory &&traj) = default;
  trajectory &operator=(trajectory &&traj) = default;

  trajectory find_reference_subtrajectory(const State &current_state, std::size_t passed_waypoints) const
  {
    const auto reference_state = find_reference_state(current_state, passed_waypoints);
    std::vector<trajectory_step<State>> sublist{ reference_state, steps_.end() };
    return { sublist, time_step_s_ };
  }

  auto find_reference_state(const State &state, const std::size_t passed_waypoints) const
  {
    if (steps_.empty())
    {
      return steps_.end();
    }

    auto best_so_far = steps_.begin();
    double distance = HUGE_VAL;
    int i = 0;

    for (auto it = steps_.begin(); it != steps_.end(); ++it)
    {
      if (it->passed_waypoints() < passed_waypoints)
      {
        continue;
      }
      else if (it->passed_waypoints() > passed_waypoints + 1)
      {  // do not skip a waypoint
        break;
      }
      ++i;
      double next_distance = state.distance_to(it->state());
      if (next_distance < distance)
      {
        best_so_far = it;
        distance = next_distance;
      }
    }

    return best_so_far;
  }

  const std::vector<trajectory_step<State>> &steps() const
  {
    return steps_;
  }
  bool empty() const
  {
    return steps_.empty();
  }
  bool is_valid() const
  {
    return !steps_.empty();
  }
  double time_step() const
  {
    return time_step_s_;
  }
  double total_distance() const
  {
    return distance_;
  }
};

}  // namespace racer
