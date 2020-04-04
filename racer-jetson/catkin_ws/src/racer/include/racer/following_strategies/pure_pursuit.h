#pragma once

#include <iostream>

#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/trajectory.h"
#include "racer/track/racing_line.h"

namespace racer::following_strategies
{

template <typename State>
class pure_pursuit
{
public:
  pure_pursuit(
      double wheelbase,
      double min_lookahead,
      double max_lookahead,
      double max_rpm)
      : wheelbase_(wheelbase),
        min_lookahead_(min_lookahead),
        max_lookahead_(max_lookahead),
        rom_to_lookahead_coef_(2 * max_lookahead / max_rpm)
  {
  }

  racer::math::angle select_steering_angle(
      const State state,
      const racer::vehicle_configuration target) const
  {
    auto diff = target.location() - calculate_rear_axle_center(state.configuration());
    double alpha = diff.angle() - state.configuration().heading_angle();

    return atan(2 * wheelbase_ * sin(alpha) / calculate_lookahead(state));
  }

  racer::vehicle_configuration find_target(
      const State &state,
      int passed_waypoints,
      const trajectory<State> &trajectory) const
  {
    const auto sub_trajectory = trajectory.find_reference_subtrajectory(state, passed_waypoints);
    if (sub_trajectory.steps().empty())
    {
      return state.configuration();
    }
    
    const auto closest = sub_trajectory.steps().front().position();
    double lookahead_sq = calculate_lookahead_sq(state);

    auto target = sub_trajectory.steps().begin();
    while (target != sub_trajectory.steps().end()
      && target->state().position().distance_sq(closest) < lookahead_sq)
    {
      ++target;
    }

    if (target == sub_trajectory.steps().end())
    {
      target = sub_trajectory.steps().back();
    }
    
    return target->state().configuration();
  }

  racer::vehicle_configuration find_target(
      const State &state,
      const racer::track::racing_line &racing_line) const
  {
    const auto vehicle_pos = state.position();
    const auto closest = racing_line.closest_point_along_the_spline(vehicle_pos);

    double lookahead_sq = calculate_lookahead_sq(state);

    do
    {
      const auto reference = racing_line.spline();
      for (std::size_t i{0}; i < reference.size(); ++i) {
        const auto j = (closest + i) % reference.size();
        const auto dist_sq = reference[closest].coordinate.distance_sq(reference[j].coordinate);
        if (dist_sq > lookahead_sq) {
          const auto prev = j == 0 ? reference.size() - 1 : j - 1;
          const auto next = (j + 1) % reference.size();
          const auto heading = reference[next].coordinate - reference[prev].coordinate;

          return {reference[j].coordinate, heading.angle()};
        }
      }

      // If we get to this point, the lookahead is too large!
      // Reduce it by half and try again!
      lookahead_sq *= 0.5;
    }
    while (lookahead_sq > 0.01);

    // We should never get to this point in practice.
    return state.configuration();
  }

private:
  const double wheelbase_;
  const double min_lookahead_;
  const double max_lookahead_;
  const double rom_to_lookahead_coef_;

  inline double calculate_lookahead_sq(const State& state) const
  {
    const double lookahead = calculate_lookahead(state);
    return lookahead * lookahead;
  }

  inline double calculate_lookahead(const State& state) const
  {
    return std::max(min_lookahead_, std::min(max_lookahead_, double(state.motor_rpm() * rom_to_lookahead_coef_)));
  }

  inline racer::math::point calculate_rear_axle_center(const racer::vehicle_configuration &cfg) const
  {
    auto rear_wheel_offset = racer::math::vector(-wheelbase_ / 2, 0).rotate(cfg.heading_angle());
    return cfg.location() + rear_wheel_offset;
  }
};

} // namespace racer::following_strategies
