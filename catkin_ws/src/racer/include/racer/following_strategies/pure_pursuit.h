#pragma once

#include <iostream>

#include "racer/math/angle.h"
#include "racer/math/point.h"
#include "racer/math/vector.h"
#include "racer/vehicle/configuration.h"
#include "racer/vehicle/motor_model.h"
#include "racer/vehicle/trajectory.h"

namespace racer::following_strategies {
template <typename State> class pure_pursuit {
public:
  pure_pursuit(double wheelbase) : wheelbase_{wheelbase} {}

  racer::math::angle
  select_steering_angle(const State state,
                        const racer::vehicle::configuration target) const {
    auto rear_axle_center = calculate_rear_axle_center_from_cog(state.cfg());

    auto to_target = target.location() - rear_axle_center;
    auto alpha = to_target.angle() - state.cfg().heading_angle();
    auto delta =
        std::atan2(2 * wheelbase_ * sin(alpha), sqrt(to_target.length()));

    return delta;
  }

private:
  const double wheelbase_;

  inline racer::math::point calculate_rear_axle_center_from_cog(
      const racer::vehicle::configuration &cfg) const {
    auto rear_wheel_offset =
        racer::math::vector(-wheelbase_ / 2, 0).rotate(cfg.heading_angle());
    return cfg.location() + rear_wheel_offset;
  }
};

} // namespace racer::following_strategies
