#include <ros/ros.h>

#include "OdometrySubject.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

OdometrySubject::OdometrySubject(
    const VehicleModel& vehicle_model,
    const std::string& odometry_frame,
    const std::string& base_link,
    tf::TransformBroadcaster& transform_broadcaster,
    ros::Publisher& odometry_topic)
    : vehicle_model_(vehicle_model),
      base_link_(base_link),
      odometry_frame_(odometry_frame),
      transform_broadcaster_(transform_broadcaster),
      odometry_topic_(odometry_topic),
      total_distance_(-1),
      total_distance_last_time_(-1),
      steering_angle_(0),
      direction_(1)
{
    last_update_time_ = ros::Time::now().toSec();

    state_ = VehicleState();
    state_.x = 0;
    state_.y = 0;
    state_.heading_angle = 0;
    state_.vx = 0;
    state_.vy = 0;
    state_.angular_velocity = 0;
}

void OdometrySubject::process_steering_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    steering_angle_ = vehicle_model_.max_steering_angle * -msg->angular.z;
    direction_ = msg->linear.x >= 0.0 : 1.0 : -1.0;    
}

void OdometrySubject::process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg)
{
    double revolutions = msg->data;
    total_distance_ = revolutions * 2 * M_PI * vehicle_model_.rear_wheel_radius;

    // first odometry message - reset the distance from the wheel encoders:
    if (total_distance_last_time_ < 0) {
        total_distance_last_time_ = total_distance_;
    }
}

void OdometrySubject::publish_odometry()
{
    std::lock_guard<std::mutex> guard(lock_);

    double current_time = ros::Time::now().toSec();
    double elapsed_time = current_time - last_update_time_;

    double step = direction_ * (total_distance_ - total_distance_last_time_);

    vehicle_model_.update_state(state_, step, steering_angle_, elapsed_time);
    publish_state_estimate(state_);

    total_distance_last_time_ = total_distance_;
    last_update_time_ = ros::Time::now().toSec();
}

void OdometrySubject::publish_state_estimate(const VehicleState &state) const
{
    if (total_distance_last_time_ < 0) {
      return;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state.heading_angle);

    ros::Time current_time = ros::Time::now();

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odometry_frame_;
    odom_trans.child_frame_id = base_link_;

    odom_trans.transform.translation.x = state.x;
    odom_trans.transform.translation.y = state.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    transform_broadcaster_.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odometry_frame_;

    //set the position
    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = base_link_;
    odom.twist.twist.linear.x = sqrt(state.vx * state.vx + state.vy * state.vy);
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = state.angular_velocity;

    //set covariance
    odom.pose.covariance[0] = 0.2; // x
    odom.pose.covariance[7] = 0.2; // y
    odom.pose.covariance[35] = 0.4; // yaw

    //publish the message
    odometry_topic_.publish(odom);
}
