#include <iostream>
#include <ros/ros.h>

#include "VehicleModel.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

VehicleModel::VehicleModel(double r, double wb, double max_delta)
    : rear_wheel_radius(r),
      wheelbase(wb),
      max_steering_angle(max_delta)
{
}

inline double integrate(double dx, double dt) {
    return dx * dt;
}

inline double fix_angle(double angle) {
    while (angle > 2 * M_PI) angle -= 2 * M_PI;
    while (angle < 0) angle += 2 * M_PI;
    return angle;
}

void VehicleModel::update_state(
    VehicleState& state,
    const double step,
    const double steering_angle,
    const double dt) const
{
    double v = step / dt;

    double slip_angle = fix_angle(atan(0.5 * tan(fix_angle(steering_angle))));
    double dx = v * cos(state.heading_angle + slip_angle);
    double dy = v * sin(state.heading_angle + slip_angle);
    double dtheta = v * cos(slip_angle) * tan(fix_angle(steering_angle)) / wheelbase;

    // std::cout << " delta: " << steering_angle  << " dx: " << dx << " dy: " << dy << " beta: " << slip_angle << " dtheta: " << dtheta << " v: " << v  << std::endl;

    state.x += integrate(dx, dt);
    state.y += integrate(dy, dt);
    state.heading_angle += integrate(dtheta, dt);
    state.vx = dx;
    state.vy = dy;
    state.angular_velocity = dtheta;
}
