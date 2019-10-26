#pragma once

#include <iostream>

#include "../action.h"

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
    virtual double speed_in_state(const State& state) const = 0;
};

} // namespace racer::vehicle_model
