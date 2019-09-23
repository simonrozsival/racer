#ifndef VEHICLE_MODEL_H_
#define VEHICLE_MODEL_H_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "VehicleState.h"

class VehicleModel
{
  public:
    const double max_steering_angle, rear_wheel_radius, wheelbase;

    VehicleModel(double r, double wb, double max_delta);

    void update_state(
      VehicleState& state,
      const double step,
      const double steering_angle,
      const double elapsed_time) const;
};

#endif
