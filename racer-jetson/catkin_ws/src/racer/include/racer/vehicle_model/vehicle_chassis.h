#pragma once

#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

namespace racer::vehicle_model
{

struct vehicle_chassis
{
    const double distance_of_center_of_gravity_to_rear_axle, wheelbase;
    const double width, length;
    const double wheel_radius;
    const double motor_to_wheel_gear_ratio;

    const std::unique_ptr<steering_servo_model> steering_servo;
    const std::unique_ptr<motor_model> motor;

    vehicle_chassis(
        double cog_offset,
        double wheelbase,
        double width,
        double length,
        double wheel_radius,
        double motor_to_wheel_gear_ratio,
        std::unique_ptr<steering_servo_model> steering_servo,
        std::unique_ptr<motor_model> motor)
        : distance_of_center_of_gravity_to_rear_axle(cog_offset),
          wheelbase(wheelbase),
          width(width),
          length(length),
          wheel_radius(wheel_radius),
          motor_to_wheel_gear_ratio(motor_to_wheel_gear_ratio),
          steering_servo(std::move(steering_servo)),
          motor(std::move(motor))
    {
    }

    vehicle_chassis(const vehicle_chassis &other) = delete;
    vehicle_chassis &operator=(const vehicle_chassis &other) = delete;

    vehicle_chassis(vehicle_chassis &&other) = delete;
    vehicle_chassis &operator=(vehicle_chassis &&other) = delete;

    static std::unique_ptr<vehicle_chassis> rc_beast()
    {
        return std::make_unique<vehicle_chassis>(
            0.155, // cog_offset
            0.31,  // wheelbase
            0.30,  // safe width
            0.45,  // safe length
            0.05,  // wheel radius
            9.0,   // wheel to motor ratio
            steering_servo_model::with_fitted_values(),
            motor_model::with_fitted_values());
    }

    inline double radius() const
    {
        double dx = length / 2.0;
        double dy = width / 2.0;
        return sqrt(dx * dx + dy * dy);
    }
};

} // namespace racer::vehicle_model
