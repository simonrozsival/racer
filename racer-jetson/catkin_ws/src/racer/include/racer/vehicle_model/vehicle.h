#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

namespace racer::vehicle_model
{

struct vehicle
{
    double distance_of_center_of_gravity_to_rear_axle, wheelbase;
    double width, length;
    double wheel_radius;

    std::unique_ptr<steering_servo_model> steering_servo;
    std::unique_ptr<motor_model> motor;

    vehicle(
        double cog_offset,
        double wheelbase,
        double width,
        double length,
        double wheel_radius,
        std::unique_ptr<steering_servo_model> steering_servo,
        std::unique_ptr<motor_model> motor)
        : distance_of_center_of_gravity_to_rear_axle(cog_offset),
          wheelbase(wheelbase),
          width(width),
          length(length),
          wheel_radius(wheel_radius),
          steering_servo(std::move(steering_servo)),
          motor(std::move(motor))
    {
    }

    vehicle(const vehicle &other) = delete;
    vehicle &operator=(const vehicle &other) = delete;

    vehicle(vehicle &&other) = default;
    vehicle &operator=(vehicle &&other) = default;

    static vehicle rc_beast()
    {
        return {
            0.155, // cog_offset
            0.31,  // wheelbase
            0.35,  // safe width
            0.55,  // safe length
            0.1,   // wheel radius
            steering_servo_model::with_fitted_values(),
            motor_model::with_fitted_values()};
    }

    inline double radius() const
    {
        double dx = length / 2;
        double dy = width / 2;
        return sqrt(dx * dx + dy * dy);
    }

    double maximum_theoretical_speed() const
    {
        return motor->max_rpm() * wheel_radius;
    }
};

} // namespace racer::vehicle_model

#endif
