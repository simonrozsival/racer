#ifndef VEHICLE_MODEL_H_
#define VEHICLE_MODEL_H_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "VehicleState.h"

#define DRIVE_BUTTON 0
#define THROTTLE_AXIS 1
#define STEERING_AXIS 0

class VehicleModel
{
  public:
    VehicleModel(
      double rear_wheel_radius,
      double wheelbase,
      double max_steering_angle);

    void update_state(
      VehicleState& state,
      const double step,
      const double steering_angle,
      const double elapsed_time);

    const double max_steering_angle() {
      return max_steering_angle_;
    }

  private:
    double rear_wheel_radius_;
    double wheelbase_;
    double max_steering_angle_;
};

#endif