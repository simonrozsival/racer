#pragma once

#include <iostream>

#define _USE_MATH_DEFINES
#include <array>
#include <cmath>

#include "racer/math.h"
#include "racer/vehicle_model/base_model.h"

using racer::math::angle;

namespace racer::vehicle_model
{
class steering_servo_model : public base_model<racer::math::angle>
{
private:
  const racer::math::angle max_steering_angle_left_, max_steering_angle_right_;

  const std::array<double, 2> angle_to_signal_;
  const std::array<double, 2> signal_adjustment_coefficients_;

public:
  steering_servo_model(racer::math::angle max_steering_angle_left, racer::math::angle max_steering_angle_right,
                       std::array<double, 2> angle_to_signal, std::array<double, 2> signal_adjustment_coefficients)
      : max_steering_angle_left_{max_steering_angle_left}, max_steering_angle_right_{max_steering_angle_right}, angle_to_signal_{angle_to_signal}, signal_adjustment_coefficients_{signal_adjustment_coefficients}
  {
  }

  static auto rc_beast()
  {
    // "signal" is the PWM and it ranges between 1200 and 1800 microseconds
    return std::make_unique<steering_servo_model>(angle::from_degrees(21.16), angle::from_degrees(-26.57),
                                                  std::array<double, 2>{12.701, 1476.686},
                                                  std::array<double, 2>{0.000329, 0.1174});
  }

  static auto simulator()
  {
    // "signal" is the direct input to the simulated actuator and it ranges
    // between -1 and 1
    return std::make_unique<steering_servo_model>(angle::from_degrees(12), angle::from_degrees(-12),
                                                  std::array<double, 2>{12.701, 1476.686},
                                                  std::array<double, 2>{0.000329, 0.1174});
  }

  angle predict_next_state(const angle &current_steering_angle, const racer::action &action, const double dt) const
  {
    const angle target_angle = target_steering_angle(action);
    const angle angle_distance =
        std::min(target_angle - current_steering_angle, 2 * M_PI - (target_angle - current_steering_angle));
    const double angle_change_rate = angle_distance / time_to_adjust(current_steering_angle, target_angle);
    return current_steering_angle + angle_change_rate * dt;
  }

  angle target_steering_angle(const racer::action &action) const
  {
    const double target_steering_angle_percentage = action.target_steering_angle();
    const angle left_or_right =
        target_steering_angle_percentage > 0 ? max_steering_angle_left_ : max_steering_angle_right_;
    return left_or_right * std::abs(target_steering_angle_percentage);
  }

  double action_input_for_angle(const angle &target) const
  {
    const angle max = target > 0 ? max_steering_angle_left_ : max_steering_angle_right_;
    return std::clamp(std::abs(target) / max, -1.0, 1.0);
  }

  angle max_steering_angle() const
  {
    return max_steering_angle_right_;
  }

private:
  inline double signal(const angle alpha) const
  {
    return angle_to_signal_[0] * alpha.to_degrees() + angle_to_signal_[1];
  }

  inline double time_to_adjust(const angle alpha, const angle beta) const
  {
    double signal_distance = std::abs(signal(alpha) - signal(beta));
    return signal_adjustment_coefficients_[0] * signal_distance + signal_adjustment_coefficients_[1];
  }
};

} // namespace racer::vehicle_model