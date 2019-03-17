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
    last_update_time_ = ros::Time::now();
}

void VehicleModel::process_steering_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    steering_angle_ = max_steering_angle_ * msg.angular.z;    
}

void VehicleModel::process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg)
{
    double revolutions = msg.data;
    double distance = revolutions * rear_wheel_radius_ * M_PI;
    double current_time = ros::Time::now();
    
    double step = distance - total_distance_;
    double elapsed_time = current_time - last_update_time_;

    estimate_next_position(step, elapsed_time);
    publish_state_estimate(last_estimated_state_);

    total_distance_ = distance;
    last_update_time_ = current_time;
}

void VehicleModel::update_state(
    VehicleState& state,
    const double step,
    const double steering_angle,
    const double elapsed_time)
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
