#pragma once

#include "racer/vehicle/motor_model.h"
#include "racer/vehicle/simulated_motor_model.h"
#include "racer/vehicle/steering_servo_model.h"

namespace racer::vehicle
{
  struct chassis
  {
    const double distance_of_center_of_gravity_to_rear_axle, wheelbase;
    const double width, length;
    const double wheel_radius;
    const double motor_to_wheel_gear_ratio;

    const std::unique_ptr<steering_servo_model> steering_servo;
    const std::unique_ptr<base_motor_model> motor;

    chassis(double cog_offset, double wheelbase, double width,
                    double length, double wheel_radius,
                    double motor_to_wheel_gear_ratio,
                    std::unique_ptr<steering_servo_model> steering_servo,
                    std::unique_ptr<base_motor_model> motor)
        : distance_of_center_of_gravity_to_rear_axle(cog_offset),
          wheelbase(wheelbase), width(width), length(length),
          wheel_radius(wheel_radius),
          motor_to_wheel_gear_ratio{motor_to_wheel_gear_ratio},
          steering_servo(std::move(steering_servo)),
          motor(std::move(motor)) {}

    chassis(const chassis &other) = delete;
    chassis &operator=(const chassis &other) = delete;

    chassis(chassis &&other) = delete;
    chassis &operator=(chassis &&other) = delete;

    static std::unique_ptr<chassis> rc_beast()
    {
      return std::make_unique<chassis>(0.155, // cog_offset
                                               0.31,  // wheelbase
                                               0.30,  // safe width
                                               0.45,  // safe length
                                               0.05,  // wheel radius
                                               9.00,
                                               steering_servo_model::rc_beast(),
                                               motor_model::rc_beast());
    }

    static std::unique_ptr<chassis> simulator()
    {
      return std::make_unique<chassis>(
          0.16,  // cog_offset
          0.32,  // wheelbase
          0.40,  // safe width
          0.60,  // safe length
          0.05,  // wheel radius
          18.00, // motor to wheel gear ratio
                 // steering_servo_model::simulator(),
                 // simulated_motor_model::with_fitted_values());
          steering_servo_model::simulator(), motor_model::simulator());
    }

    inline double radius() const
    {
      double dx = length / 2.0;
      double dy = width / 2.0;
      return sqrt(dx * dx + dy * dy);
    }
  };

} // namespace racer::vehicle
