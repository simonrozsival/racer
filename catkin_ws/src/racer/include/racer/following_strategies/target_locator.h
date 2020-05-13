#pragma once

#include <iostream>

#include "racer/vehicle/trajectory.h"

namespace racer::following_strategies {

template <typename State>
class target_locator {
public:
  target_locator(double min_lookahead, double max_lookahead, double max_rpm)
      : min_lookahead_(min_lookahead), max_lookahead_(max_lookahead),
        rpm_to_lookahead_coef_(2 * max_lookahead / max_rpm) {}

  State find_target(const State &state, int passed_waypoints,
                    const racer::vehicle::trajectory<State> &trajectory) const {
    auto sub_trajectory =
        trajectory.find_reference_subtrajectory(state, passed_waypoints);
    if (sub_trajectory.steps().empty()) {
      return state.cfg();
    }

    const auto closest = sub_trajectory.steps().front().state().position();
    double lookahead_sq = calculate_lookahead_sq(state);

    auto target = sub_trajectory.steps().begin();
    while (target != sub_trajectory.steps().end() &&
           target->state().position().distance_sq(closest) < lookahead_sq) {
      ++target;
    }

    if (target == sub_trajectory.steps().end()) {
      return sub_trajectory.steps().back().state();
    }

    return target->state();
  }

private:
  const double min_lookahead_;
  const double max_lookahead_;
  const double rpm_to_lookahead_coef_;

  inline double calculate_lookahead_sq(const State &state) const {
    const double lookahead = calculate_lookahead(state);
    return lookahead * lookahead;
  }

  inline double calculate_lookahead(const State &state) const {
    return std::max(min_lookahead_,
                    std::min(max_lookahead_, double(state.motor_rpm() *
                                                    rpm_to_lookahead_coef_)));
  }
};

} // namespace racer::following_strategies
