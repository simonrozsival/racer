#pragma once

#include <iostream>

#include "racer/action.h"
#include "racer/vehicle_model/rpm.h"

namespace racer::vehicle_model
{

    template <typename State>
    class base_model
    {
    public:
        virtual ~base_model() = default;
        virtual State predict_next_state(const State &state, const action &action, const double dt) const = 0;
    };

    template <typename State>
    class vehicle_model : public base_model<State>
    {
    public:
        virtual ~vehicle_model() = default;
        virtual double maximum_theoretical_speed() const = 0;
        virtual double speed_in_state(const State &state) const = 0;
    };

    class base_motor_model : public base_model<rpm>
    {
    public:
        virtual double normalize_rpm(const rpm raw_rpm) const = 0;
        virtual rpm max_rpm() const = 0;
    };

} // namespace racer::vehicle_model
