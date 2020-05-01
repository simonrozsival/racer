#pragma once

#include <iostream>
#include <memory>

#define _USE_MATH_DEFINES
#include <array>
#include <cmath>

#include "racer/math.h"
#include "racer/vehicle_model/base_model.h"

using racer::math::angle;

namespace racer::vehicle_model {
class simulated_steering_servo_model : public base_model<racer::math::angle> {
private:
  const std::array<double, 2> coeffs_;
  const double tolerance_;

public:
  simulated_steering_servo_model(const std::array<double, 2> coeffs,
                                 const double tolerance)
      : coeffs_{coeffs}, tolerance_{tolerance} {}

  static auto with_fitted_values() {
    return std::make_unique<simulated_steering_servo_model>(
        std::array<double, 2>{0.2, 0.4}, 0.05);
  }

  angle predict_next_state(const angle &current_steering_angle,
                           const racer::action &action, const double dt) const {
    const double current = clamp(current_steering_angle);
    const double target = clamp(action.target_steering_angle());

    double difference = target - current;
    const double undershoot = difference > 0.0 ? -tolerance_ : tolerance_;
    difference += undershoot;

    const double distance = std::abs(difference);
    const double time_to_adjust = coeffs_[0] * distance + coeffs_[1];
    const double change = (difference / time_to_adjust) * dt;
    return distance < tolerance_ ? current
                                 : clamp(current_steering_angle + change);
  }

private:
  inline double clamp(const double percentage) const {
    return std::clamp(percentage, -1.0, 1.0);
  }
};

} // namespace racer::vehicle_model