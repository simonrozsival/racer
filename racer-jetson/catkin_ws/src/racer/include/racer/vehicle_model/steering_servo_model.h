#pragma once

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <array>

#include "racer/vehicle_model/base_model.h"
#include "racer/math.h"

using racer::math::angle;

namespace racer::vehicle_model
{

class steering_servo_model : public base_model<racer::math::angle>
{
private:
    const racer::math::angle max_steering_angle_left_, max_steering_angle_right_;
    const std::array<double, 2> angle_to_pwm_;
    const std::array<double, 2> pwm_adjustment_coefficients_;

public:
    steering_servo_model(
        racer::math::angle max_steering_angle_left,
        racer::math::angle max_steering_angle_right,
        std::array<double, 2> angle_to_pwm,
        std::array<double, 2> pwm_adjustment_coefficients)
        : max_steering_angle_left_{max_steering_angle_left},
          max_steering_angle_right_{max_steering_angle_right},
          angle_to_pwm_{angle_to_pwm},
          pwm_adjustment_coefficients_{pwm_adjustment_coefficients}
    {
    }

    static auto with_fitted_values()
    {
        return std::make_unique<steering_servo_model>(
            angle::from_degrees(-21.16),
            angle::from_degrees(26.57),
            std::array<double, 2>{12.701, 1476.686},
            std::array<double, 2>{0.000329, 0.1174});
    }

    angle predict_next_state(const angle &current_steering_angle, const racer::action &action, const double dt) const
    {
        const angle target_angle = target_steering_angle(action);
        const double angle_distance = current_steering_angle.distance_to(target_angle);
        const double angle_change_rate = direction_of_change(current_steering_angle, target_angle) * angle_distance / time_to_adjust(current_steering_angle, target_angle);
        return current_steering_angle + angle_change_rate * dt;
    }

private:
    inline double pwm(const angle alpha) const
    {
        return angle_to_pwm_[0] * alpha.to_degrees() + angle_to_pwm_[1];
    }

    inline double time_to_adjust(const angle alpha, const angle beta) const
    {
        double pwm_distance = std::abs(pwm(alpha) - pwm(beta));
        return pwm_adjustment_coefficients_[0] * pwm_distance + pwm_adjustment_coefficients_[1];
    }

    angle target_steering_angle(const racer::action &action) const
    {
        const double target_steering_angle_percentage = action.target_steering_angle();
        return target_steering_angle_percentage > 0
                   ? max_steering_angle_right_ * target_steering_angle_percentage
                   : max_steering_angle_left_ * target_steering_angle_percentage;
    }

    double direction_of_change(const angle angle_from, const angle angle_to) const
    {
        const double delta = angle_to.to_angle_around_zero() - angle_from.to_angle_around_zero();
        return racer::math::sign(delta);
    }
};

} // namespace racer::vehicle_model