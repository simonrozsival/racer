#include "VehicleModel.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

VehicleModel::VehicleModel(
    double rear_wheel_radius,
    double wheelbase,
    double max_steering_angle)
    : rear_wheel_radius_(rear_wheel_radius),
      wheelbase_(wheelbase),
      max_steering_angle_(max_steering_angle)
{
}

void VehicleModel::update_state(
    VehicleState& state,
    const double step,
    const double steering_angle,
    const double elapsed_time) const
{
    double dx = cos(state.heading_angle) * step;;
    double dy = sin(state.heading_angle) * step;
    double dtheta = tan(state.heading_angle) / wheelbase_ * step;
    
    state.x += dx;
    state.y += dy;
    state.heading_angle += dtheta;
    state.vx = dx / elapsed_time;
    state.vy = dy / elapsed_time;
    state.angular_velocity = dtheta / elapsed_time;
}
