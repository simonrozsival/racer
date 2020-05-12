#pragma once

#include <algorithm>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>


#include "racer/vehicle/kinematic/state.h"

#include "racer/math.h"
#include "racer/vehicle/action.h"
#include "racer/vehicle/base_model.h"
#include "racer/vehicle/chassis.h"
#include "racer/vehicle/configuration.h"
#include "racer/vehicle/motor_model.h"
#include "racer/vehicle/steering_servo_model.h"

namespace racer::vehicle::kinematic
{
  class model : public base_vehicle_model<state>
  {
  public:
    const std::shared_ptr<racer::vehicle::chassis> chassis;

    model(std::shared_ptr<racer::vehicle::chassis> chassis) : chassis{chassis}
    {
    }

  public:
    state predict_next_state(const state &current, const action &input, const double dt) const override
    {
      rpm motor_rpm = chassis->motor->predict_next_state(current.motor_rpm(), input, dt);
      racer::math::angle steering_angle =
          chassis->steering_servo->predict_next_state(current.steering_angle(), input, dt);

      // the computed engine RPM is used immediately - not with a 1 step delay
      // - otherwise, the car could not start moving in the very first step (with
      // the current implementation)

      double v = calculate_speed_with_no_slip_assumption(motor_rpm);

      // just some renaming to make the equations look the same as in the thesis
      double theta = current.cfg().heading_angle();
      double delta = steering_angle;
      double beta = slip_angle(steering_angle);
      double L = chassis->wheelbase;

      configuration translation_and_rotation{v * cos(theta + beta) * dt, v * sin(theta + beta) * dt,
                                                     v * cos(beta) * tan(delta) / L * dt};

      return {current.cfg() + translation_and_rotation, motor_rpm, steering_angle};
    }

    double maximum_theoretical_speed() const override
    {
      const double motor_revolutions_per_second = chassis->motor->max_rpm() / 60.0;
      const double wheel_revolutions_per_second = motor_revolutions_per_second / chassis->motor_to_wheel_gear_ratio;
      return wheel_revolutions_per_second * 2 * M_PI * chassis->wheel_radius;
    }

    double calculate_speed_with_no_slip_assumption(const rpm &motor_rpm) const
    {
      return top_speed_ * (motor_rpm / top_rpm_);
    }

    double speed_in_state(const state &state) const
    {
      return top_speed_ * (state.motor_rpm() / top_rpm_);
    }

    action action_from_speed_and_steering_angle(const double speed, const double angle) const
    {
      return {std::clamp(speed / top_speed_, 0.0, 1.0), chassis->steering_servo->action_input_for_angle(angle)};
    }

    double top_speed() const
    {
      return top_speed_;
    }

  private:
    const double top_speed_{4.1};
    const double top_rpm_{14000};

    double slip_angle(const angle steering_angle) const
    {
      return atan((chassis->distance_of_center_of_gravity_to_rear_axle / chassis->wheelbase) * tan(steering_angle));
    }
  };

} // namespace racer::vehicle::kinematic
