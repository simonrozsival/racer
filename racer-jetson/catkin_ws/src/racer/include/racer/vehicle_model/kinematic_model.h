#ifndef KINEMATIC_MODEL_H_
#define KINEMATIC_MODEL_H_

#include <iostream>
#include <algorithm>
#include <cmath>

#include "racer/vehicle_configuration.h"

#include "racer/action.h"
#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/base_model.h"

namespace racer::vehicle_model::kinematic
{

struct state
{
private:
    vehicle_configuration configuration_;
    double speed_, steering_angle_;

public:
    state()
        : configuration_{},
          speed_{0},
          steering_angle_{0}
    {
    }

    state(vehicle_configuration configuration, double speed, double steering_angle)
        : configuration_(configuration), speed_(speed), steering_angle_(steering_angle)
    {
    }

    state(const state &s) = default;
    state &operator=(const state &s) = default;

    state(state &&s) = default;
    state &operator=(state &&s) = default;

    const vehicle_configuration &configuration() const { return configuration_; }
    inline const racer::math::point &position() const { return configuration_.location(); }
    const double &speed() const { return speed_; }
    const double &steering_angle() const { return steering_angle_; }

    bool operator==(const state &other) const
    {
        return configuration_ == other.configuration_ && speed_ == other.speed_ && steering_angle_ == other.steering_angle_;
    }

    bool operator!=(const state &other) const
    {
        return !(*this == other);
    }

    double distance_to(const state &other) const
    {
        return (configuration_.location() - other.configuration_.location()).length();
    }

    bool is_valid() const { return configuration_.is_valid(); }
};

class model : public racer::vehicle_model::base_model<state>
{
private:
    const vehicle vehicle_;
    const double dt_;

public:
    model(const vehicle &vehicle, const double dt)
        : vehicle_{vehicle}, dt_{dt}
    {
    }

public:
    state predict_next_state(const state &current, const action &input) const override
    {
        double speed = calculate_speed(current, input);
        double steering_angle = calculate_steering_angle(current, input);
        racer::math::angle slip_angle = atan((vehicle_.distance_of_center_of_gravity_to_rear_axle / vehicle_.wheelbase) * tan(racer::math::angle(steering_angle)));

        vehicle_configuration configration_step{
            speed * cos(current.configuration().heading_angle() + slip_angle) * dt_,
            speed * sin(current.configuration().heading_angle() + slip_angle) * dt_,
            speed * cos(slip_angle) * tan(steering_angle) / vehicle_.wheelbase * dt_};

        return {current.configuration() + configration_step, speed, steering_angle};
    }

private:
    double calculate_speed(const state &current, const action &input) const
    {
        double target_speed = input.throttle() * vehicle_.max_speed;
        double acceleration = sign(target_speed - current.speed()) * vehicle_.max_acceleration;
        double speed_delta = acceleration * dt_;

        return abs(current.speed() - target_speed) < abs(speed_delta)
                   ? target_speed // don't overshoot
                   : std::min(vehicle_.max_speed, std::max(vehicle_.max_reversing_speed, current.speed() + speed_delta));
    }

    double calculate_steering_angle(const state &current, const action &input) const
    {
        double target_steering_angle = input.target_steering_angle() * vehicle_.max_steering_angle;
        double speed = sign(target_steering_angle - current.steering_angle()) * vehicle_.max_steering_speed;
        double delta = speed * dt_;

        return abs(target_steering_angle - current.steering_angle()) < abs(speed)
                   ? target_steering_angle
                   : std::min(vehicle_.max_steering_angle, std::max(-vehicle_.max_steering_angle, current.steering_angle() + delta));
    }

    inline constexpr double sign(double x) const
    {
        return (0 < x) - (x < 0);
    }
};

} // namespace racer::vehicle_model::kinematic

#endif
