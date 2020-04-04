#pragma once

#include <iostream>

#include "racer/math.h"
#include "racer/track/racing_line.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"

namespace racer::following_strategies
{
template <typename State>
class target_locator
{
public:
  target_locator(double min_lookahead, double max_lookahead, double max_rpm)
    : min_lookahead_(min_lookahead), max_lookahead_(max_lookahead), rpm_to_lookahead_coef_(2 * max_lookahead / max_rpm)
  {
  }

  State find_target(const State &state, int passed_waypoints, const trajectory<State> &trajectory) const
  {
    auto sub_trajectory = trajectory.find_reference_subtrajectory(state, passed_waypoints);
    if (sub_trajectory.steps().empty())
    {
      return state.configuration();
    }

    const auto closest = sub_trajectory.steps().front().state().position();
    double lookahead_sq = calculate_lookahead_sq(state);

    auto target = sub_trajectory.steps().begin();
    while (target != sub_trajectory.steps().end() && target->state().position().distance_sq(closest) < lookahead_sq)
    {
      ++target;
    }

    if (target == sub_trajectory.steps().end())
    {
      return sub_trajectory.steps().back().state();
    }

    return target->state();
  }

  racer::vehicle_configuration find_target(const State &state, const racer::track::racing_line &racing_line) const
  {
    const auto vehicle_pos = state.position();
    const auto closest = racing_line.closest_point_along_the_spline(vehicle_pos);

    double lookahead_sq = calculate_lookahead_sq(state);

    do
    {
      const auto reference = racing_line.spline();
      for (std::size_t i{ 0 }; i < reference.size(); ++i)
      {
        const auto j = (closest + i) % reference.size();
        const auto dist_sq = reference[closest].coordinate.distance_sq(reference[j].coordinate);
        if (dist_sq > lookahead_sq)
        {
          const auto prev = j == 0 ? reference.size() - 1 : j - 1;
          const auto next = (j + 1) % reference.size();
          const auto heading = reference[next].coordinate - reference[prev].coordinate;

          return { reference[j].coordinate, heading.angle() };
        }
      }

      // If we get to this point, the lookahead is too large!
      // Reduce it by half and try again!
      lookahead_sq *= 0.5;
    } while (lookahead_sq > 0.01);

    // We should never get to this point in practice.
    return state.configuration();
  }

private:
  const double min_lookahead_;
  const double max_lookahead_;
  const double rpm_to_lookahead_coef_;

  inline double calculate_lookahead_sq(const State &state) const
  {
    const double lookahead = calculate_lookahead(state);
    return lookahead * lookahead;
  }

  inline double calculate_lookahead(const State &state) const
  {
    return std::max(min_lookahead_, std::min(max_lookahead_, double(state.motor_rpm() * rpm_to_lookahead_coef_)));
  }
};

}  // namespace racer::following_strategies
