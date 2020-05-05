#pragma once

#include <array>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/base_model.h"

namespace racer::vehicle_model
{
    class simulated_motor_model : public base_motor_model
    {
    private:
        const rpm max_rpm_;
        const double adjustment_speed_;

    public:
        simulated_motor_model(const rpm max_rpm, const double adjustment_speed)
            : max_rpm_{max_rpm}, adjustment_speed_{adjustment_speed} {}

        static auto with_fitted_values()
        {
            return std::make_unique<simulated_motor_model>(14000.0, 0.04);
        }

        rpm max_rpm() const { return max_rpm_; }

        rpm predict_next_state(const rpm &state, const action &action,
                               const double dt) const
        {
            const double normalized_rpm = normalize_rpm(state);
            const double change_rate =
                racer::math::sign(action.throttle() - normalized_rpm) *
                adjustment_speed_;
            return max_rpm_ * std::clamp(normalized_rpm + change_rate * dt, 0.0, 1.0);
        }

        double normalize_rpm(const rpm raw_rpm) const
        {
            return raw_rpm / max_rpm_;
        }
    };

} // namespace racer::vehicle_model