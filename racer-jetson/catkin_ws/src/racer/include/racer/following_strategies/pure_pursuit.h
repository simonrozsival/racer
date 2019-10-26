#pragma once

#include <iostream>

#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/trajectory.h"

namespace racer::following_strategies
{

template <typename State>
class pure_pursuit
{
public:
  pure_pursuit(
      double wheelbase,
      double min_lookahead,
      double lookahead_coefficient)
      : wheelbase_(wheelbase), min_lookahead_(min_lookahead), lookahead_coefficient_(lookahead_coefficient)
  {
  }

  racer::math::angle select_steering_angle(
      const State &current_state, int passed_waypoints,
      const racer::trajectory<State> &trajectory) const
  {
    const auto reference = find_reference_position(
        current_state, passed_waypoints, trajectory);

    auto diff = reference.location() - calculate_rear_axle_center(current_state);
    double alpha = diff.angle() - current_state.configuration().heading_angle();

    return atan(2 * wheelbase_ * sin(alpha) / calculate_lookahead(current_state));
  }

  racer::vehicle_configuration find_reference_position(
      const State &current_state,
      int passed_waypoints,
      const trajectory<State> &trajectory) const
  {
    auto sub_trajectory = trajectory.find_reference_subtrajectory(current_state, passed_waypoints);

    if (sub_trajectory.steps().empty())
    {
      return current_state.configuration();
    }

    auto rear_axle_center = calculate_rear_axle_center(current_state.configuration());

    const double lookahead = calculate_lookahead(current_state);
    const double lookahead_sq = lookahead * lookahead;
    auto reference_step = sub_trajectory.steps().begin();
    while (reference_step != sub_trajectory.steps().end()
      && reference_step->state().position().distance_sq(rear_axle_center) < lookahead_sq)
    {
      ++reference_step;
    }

    return reference_step == sub_trajectory.steps().end()
               ? sub_trajectory.steps().back().state().configuration()
               : reference_step->state().configuration();
  }

private:
  const double wheelbase_;
  const double min_lookahead_;
  const double lookahead_coefficient_;

  inline double calculate_lookahead(const State& state) const
  {
    return std::max(min_lookahead_, double(state.motor_rpm() * lookahead_coefficient_));
  }

  inline racer::math::point calculate_rear_axle_center(const State &state) const
  {
    auto rear_wheel_offset = racer::math::vector(-wheelbase_ / 2, 0).rotate(state.configuration().heading_angle());
    return state.position() + rear_wheel_offset;
  }
};

} // namespace racer::following_strategies
