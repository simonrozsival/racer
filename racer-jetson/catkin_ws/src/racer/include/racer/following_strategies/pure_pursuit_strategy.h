#pragma once

#include <iostream>

#include "racer/following_strategies/pure_pursuit.h"
#include "racer/following_strategies/target_locator.h"
#include "racer/math.h"
#include "racer/track/racing_line.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"

namespace racer::following_strategies {

template <typename State>
class pure_pursuit_strategy : public following_strategy<State> {
public:
  pure_pursuit_strategy(target_locator<State> locator,
                        pure_pursuit<State> pursuit, const double max_rpm)
      : target_locator_{locator}, pure_pursuit_{pursuit}, max_rpm_{max_rpm} {}

  racer::action select_action(
      const State &current_state, const std::size_t passed_waypoints,
      const racer::trajectory<State> &reference_trajectory,
      const std::shared_ptr<racer::occupancy_grid> map) const override {

    const auto target = target_locator_.find_target(
        current_state, passed_waypoints, reference_trajectory);

    const auto steering_angle = pure_pursuit_.select_steering_angle(
        current_state, target.configuration());
    const auto throttle = target.motor_rpm() / max_rpm_;

    return {throttle, steering_angle};
  }

private:
  const pure_pursuit<State> pure_pursuit_;
  const target_locator<State> target_locator_;
  const double max_rpm_;
};

} // namespace racer::following_strategies
