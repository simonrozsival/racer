#pragma once

#include <algorithm>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/vehicle/configuration.h"

#include "racer/vehicle/action.h"
#include "racer/math.h"

#include "racer/vehicle/base_model.h"
#include "racer/vehicle/motor_model.h"
#include "racer/vehicle/steering_servo_model.h"
#include "racer/vehicle/chassis.h"

namespace racer::vehicle::kinematic
{
  struct state
  {
  private:
    configuration configuration_;
    rpm motor_rpm_;
    racer::math::angle steering_angle_;

  public:
    state() : configuration_{}, motor_rpm_{0}, steering_angle_{0}
    {
    }

    state(configuration configuration) : state(configuration, 0, 0)
    {
    }

    state(configuration configuration, rpm motor_rpm, racer::math::angle steering_angle)
        : configuration_(configuration), motor_rpm_(motor_rpm), steering_angle_(steering_angle)
    {
    }

    state(const state &s) = default;
    state &operator=(const state &s) = default;

    state(state &&s) = default;
    state &operator=(state &&s) = default;

    inline const configuration &cfg() const
    {
      return configuration_;
    }
    inline const racer::math::point &position() const
    {
      return configuration_.location();
    }
    inline const rpm &motor_rpm() const
    {
      return motor_rpm_;
    }
    inline const racer::math::angle &steering_angle() const
    {
      return steering_angle_;
    }

    inline bool operator==(const state &other) const
    {
      return configuration_ == other.configuration_ && motor_rpm_ == other.motor_rpm_ &&
             steering_angle_ == other.steering_angle_;
    }

    inline bool operator!=(const state &other) const
    {
      return !(*this == other);
    }

    inline double distance_to(const state &other) const
    {
      return configuration_.location().distance(other.configuration_.location());
    }

    inline bool is_valid() const
    {
      return configuration_.is_valid();
    }

    friend std::ostream &operator<<(std::ostream &os, const state &s)
    {
      return os << "(theta: " << s.steering_angle() << ", rpm: " << s.motor_rpm() << ", " << s.configuration_.location()
                << ")";
    }
  };

} // namespace racer::vehicle::kinematic
